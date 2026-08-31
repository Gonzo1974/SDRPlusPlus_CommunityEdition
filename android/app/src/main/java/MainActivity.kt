package org.sdrpp.sdrpp;

import android.app.NativeActivity;
import android.app.AlertDialog;
import android.app.PendingIntent;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.pm.PackageManager;
import android.hardware.usb.*;
import android.Manifest;
import android.os.Bundle;
import android.view.View;
import android.view.KeyEvent;
import android.view.inputmethod.InputMethodManager;
import android.util.Log;
import android.content.res.AssetManager;

import androidx.core.app.ActivityCompat;

import androidx.core.content.PermissionChecker;

import java.util.concurrent.LinkedBlockingQueue;
import java.io.*;

private const val ACTION_USB_PERMISSION = "org.sdrpp.sdrpp.USB_PERMISSION";
private const val DOCUMENT_PICKER_REQUEST = 0x5344;
private const val DOCUMENT_PICKER_IDLE = 0;
private const val DOCUMENT_PICKER_PENDING = 1;
private const val DOCUMENT_PICKER_SELECTED = 2;
private const val DOCUMENT_PICKER_CANCELLED = 3;
private const val DOCUMENT_PICKER_ERROR = 4;

private val usbReceiver = object : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        if (ACTION_USB_PERMISSION == intent.action) {
            synchronized(this) {
                var _this = context as MainActivity;
                _this.SDR_device = intent.getParcelableExtra(UsbManager.EXTRA_DEVICE)
                if (intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)) {
                    _this.SDR_conn = _this.usbManager!!.openDevice(_this.SDR_device);
                    
                    // Save SDR info
                    _this.SDR_VID = _this.SDR_device!!.getVendorId();
                    _this.SDR_PID = _this.SDR_device!!.getProductId()
                    _this.SDR_FD = _this.SDR_conn!!.getFileDescriptor();
                }
                
                // Whatever the hell this does
                context.unregisterReceiver(this);

                // Hide again the system bars
                _this.hideSystemBars();
            }
        }
    }
}

class MainActivity : NativeActivity() {
    private val TAG : String = "SDR++ CE";
    public var usbManager : UsbManager? = null;
    public var SDR_device : UsbDevice? = null;
    public var SDR_conn : UsbDeviceConnection? = null;
    public var SDR_VID : Int = -1;
    public var SDR_PID : Int = -1;
    public var SDR_FD : Int = -1;

    private val documentPickerLock = Any();
    private var documentPickerStatus : Int = DOCUMENT_PICKER_IDLE;
    private var documentPickerResult : String = "";

    fun checkAndAsk(permission: String) {
        if (PermissionChecker.checkSelfPermission(this, permission) != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, arrayOf(permission), 1);
        }
    }

    public fun hideSystemBars() {
        val decorView = getWindow().getDecorView();
        val uiOptions = View.SYSTEM_UI_FLAG_HIDE_NAVIGATION or View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY;
        decorView.setSystemUiVisibility(uiOptions);
    }

    public override fun onCreate(savedInstanceState: Bundle?) {
        // Hide bars
        hideSystemBars();

        // Ask for required permissions, without these the app cannot run.
        checkAndAsk(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        checkAndAsk(Manifest.permission.READ_EXTERNAL_STORAGE);

        // TODO: Have the main code wait until these two permissions are available

        // Register events
        usbManager = getSystemService(Context.USB_SERVICE) as UsbManager;
        val permissionIntent = PendingIntent.getBroadcast(this, 0, Intent(ACTION_USB_PERMISSION), 0)
        val filter = IntentFilter(ACTION_USB_PERMISSION)
        registerReceiver(usbReceiver, filter)

        // Get permission for all USB devices
        val devList = usbManager!!.getDeviceList();
        for ((name, dev) in devList) {
            usbManager!!.requestPermission(dev, permissionIntent);
        }

        // Ask for internet permission
        checkAndAsk(Manifest.permission.INTERNET);

        super.onCreate(savedInstanceState)
    }

    public override fun onResume() {
        // Hide bars again
        hideSystemBars();
        super.onResume();
    }

    // Called from native code. The Storage Access Framework returns a content:// URI,
    // which is copied to app cache so the existing C++ file parser can remain unchanged.
    fun openDocumentPicker(): Boolean {
        var stalePath = "";
        synchronized(documentPickerLock) {
            if (documentPickerStatus == DOCUMENT_PICKER_PENDING) {
                return false;
            }
            if (documentPickerStatus == DOCUMENT_PICKER_SELECTED) {
                stalePath = documentPickerResult;
            }
            documentPickerStatus = DOCUMENT_PICKER_PENDING;
            documentPickerResult = "";
        }

        if (stalePath.isNotEmpty()) {
            File(stalePath).delete();
        }

        runOnUiThread {
            try {
                val intent = Intent(Intent.ACTION_OPEN_DOCUMENT).apply {
                    addCategory(Intent.CATEGORY_OPENABLE);
                    type = "*/*";
                    putExtra(Intent.EXTRA_MIME_TYPES, arrayOf(
                        "application/json",
                        "text/json",
                        "text/plain",
                        "application/octet-stream"
                    ));
                    addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
                }
                startActivityForResult(intent, DOCUMENT_PICKER_REQUEST);
            }
            catch (e: Exception) {
                setDocumentPickerError("Could not open the Android file picker: " +
                    (e.message ?: e.javaClass.simpleName));
            }
        }
        return true;
    }

    fun getDocumentPickerStatus(): Int {
        synchronized(documentPickerLock) {
            return documentPickerStatus;
        }
    }

    fun consumeDocumentPickerResult(): String {
        synchronized(documentPickerLock) {
            val result = documentPickerResult;
            documentPickerStatus = DOCUMENT_PICKER_IDLE;
            documentPickerResult = "";
            return result;
        }
    }

    private fun setDocumentPickerError(message: String) {
        synchronized(documentPickerLock) {
            documentPickerStatus = DOCUMENT_PICKER_ERROR;
            documentPickerResult = message;
        }
    }

    override fun onActivityResult(requestCode: Int, resultCode: Int, data: Intent?) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != DOCUMENT_PICKER_REQUEST) {
            return;
        }

        hideSystemBars();
        if (resultCode != RESULT_OK) {
            synchronized(documentPickerLock) {
                documentPickerStatus = DOCUMENT_PICKER_CANCELLED;
                documentPickerResult = "";
            }
            return;
        }

        val uri = data?.data;
        if (uri == null) {
            setDocumentPickerError("The Android file picker did not return a document.");
            return;
        }

        Thread {
            var temporaryFile : File? = null;
            try {
                val targetFile = File.createTempFile("frequency_manager_import_", ".json", cacheDir);
                temporaryFile = targetFile;
                val input = contentResolver.openInputStream(uri)
                    ?: throw IOException("The selected document could not be opened.");
                input.use { source ->
                    FileOutputStream(targetFile).use { destination ->
                        source.copyTo(destination);
                    }
                }
                synchronized(documentPickerLock) {
                    documentPickerStatus = DOCUMENT_PICKER_SELECTED;
                    documentPickerResult = targetFile.absolutePath;
                }
            }
            catch (e: Exception) {
                temporaryFile?.delete();
                setDocumentPickerError("Could not read the selected document: " +
                    (e.message ?: e.javaClass.simpleName));
            }
        }.start();
    }

    fun showSoftInput() {
        val inputMethodManager = getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager;
        inputMethodManager.showSoftInput(window.decorView, 0);
    }

    fun hideSoftInput() {
        val inputMethodManager = getSystemService(Context.INPUT_METHOD_SERVICE) as InputMethodManager;
        inputMethodManager.hideSoftInputFromWindow(window.decorView.windowToken, 0);
        hideSystemBars();
    }

    // Queue for the Unicode characters to be polled from native code (via pollUnicodeChar())
    private var unicodeCharacterQueue: LinkedBlockingQueue<Int> = LinkedBlockingQueue()

    // We assume dispatchKeyEvent() of the NativeActivity is actually called for every
    // KeyEvent and not consumed by any View before it reaches here
    override fun dispatchKeyEvent(event: KeyEvent): Boolean {
        if (event.action == KeyEvent.ACTION_DOWN) {
            unicodeCharacterQueue.offer(event.getUnicodeChar(event.metaState))
        }
        return super.dispatchKeyEvent(event)
    }

    fun pollUnicodeChar(): Int {
        return unicodeCharacterQueue.poll() ?: 0
    }

    public fun createIfDoesntExist(path: String) {
        // This is a directory, create it in the filesystem
        var folder = File(path);
        var success = true;
        if (!folder.exists()) {
            success = folder.mkdirs();
        }
        if (!success) {
            Log.e(TAG, "Could not create folder with path " + path);
        }
    }

    public fun extractDir(aman: AssetManager, local: String, rsrc: String): Int {
        val flist = aman.list(rsrc);
        var ecount = 0;
        for (fp in flist) {
            val lpath = local + "/" + fp;
            val rpath = rsrc + "/" + fp;

            Log.w(TAG, "Extracting '" + rpath + "' to '" + lpath + "'");

            // Create local path if non-existent
            createIfDoesntExist(local);
            
            // Create if directory
            val ext = extractDir(aman, lpath, rpath);

            // Extract if file
            if (ext == 0) {
                // This is a file, extract it
                val _os = FileOutputStream(lpath);
                val _is = aman.open(rpath);
                val ilen = _is.available();
                var fbuf = ByteArray(ilen);
                _is.read(fbuf, 0, ilen);
                _os.write(fbuf);
                _os.close();
                _is.close();
            }

            ecount++;
        }
        return ecount;
    }

    public fun getAppDir(): String {
        val fdir = getFilesDir().getAbsolutePath();

        // Extract all resources to the app directory
        val aman = getAssets();
        extractDir(aman, fdir + "/res", "res");
        createIfDoesntExist(fdir + "/modules");

        return fdir;
    }
}
