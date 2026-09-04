package se.g30.control;

import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.Context;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;

import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Locale;
import java.util.Queue;
import java.util.UUID;

/**
 * Small BLE client for the G30-NAV Nordic-UART-style service exposed by the
 * matching ESP32 firmware. Permission prompts are handled by MainActivity.
 */
public final class G30BleClient {

    public interface Listener {
        void onConnectionChanged(boolean connected, String message);
        void onLineReceived(String line);
        void onBleError(String message);
    }

    public static final String DEVICE_NAME = "G30-NAV";

    private static final UUID SERVICE_UUID =
            UUID.fromString("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID RX_UUID =
            UUID.fromString("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID TX_UUID =
            UUID.fromString("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
    private static final UUID CCCD_UUID =
            UUID.fromString("00002902-0000-1000-8000-00805F9B34FB");

    private static final long SCAN_TIMEOUT_MS = 10_000L;

    private final Context context;
    private final Listener listener;
    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private final BluetoothAdapter adapter;

    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic rxCharacteristic;
    private BluetoothGattCharacteristic txCharacteristic;

    private boolean scanning;
    private boolean ready;
    private boolean writeInProgress;
    private final Queue<byte[]> writeQueue = new ArrayDeque<>();

    public G30BleClient(Context context, Listener listener) {
        this.context = context.getApplicationContext();
        this.listener = listener;
        BluetoothManager manager =
                (BluetoothManager) context.getSystemService(Context.BLUETOOTH_SERVICE);
        this.adapter = manager != null ? manager.getAdapter() : null;
    }

    public boolean isBluetoothSupported() {
        return adapter != null;
    }

    @SuppressLint("MissingPermission")
    public boolean isBluetoothEnabled() {
        return adapter != null && adapter.isEnabled();
    }

    public boolean isReady() {
        return ready;
    }

    @SuppressLint("MissingPermission")
    public void scanAndConnect() {
        if (adapter == null) {
            listener.onBleError("Telefonen saknar Bluetooth.");
            return;
        }
        if (!adapter.isEnabled()) {
            listener.onBleError("Bluetooth är avstängt.");
            return;
        }

        disconnect();
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) {
            listener.onBleError("BLE-skannern kunde inte startas.");
            return;
        }

        scanning = true;
        listener.onConnectionChanged(false, "Söker efter G30-NAV …");
        scanner.startScan(scanCallback);

        mainHandler.postDelayed(() -> {
            if (scanning) {
                stopScan();
                listener.onBleError("Hittade inte G30-NAV inom 10 sekunder.");
            }
        }, SCAN_TIMEOUT_MS);
    }

    @SuppressLint("MissingPermission")
    private void stopScan() {
        if (scanner != null && scanning) {
            try {
                scanner.stopScan(scanCallback);
            } catch (SecurityException ignored) {
                // MainActivity requests the permission before this class is used.
            }
        }
        scanning = false;
    }

    @SuppressLint("MissingPermission")
    private void connect(BluetoothDevice device) {
        stopScan();
        listener.onConnectionChanged(false, "Ansluter till G30-NAV …");
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            gatt = device.connectGatt(context, false, gattCallback,
                    BluetoothDevice.TRANSPORT_LE);
        } else {
            gatt = device.connectGatt(context, false, gattCallback);
        }
    }

    @SuppressLint("MissingPermission")
    public void disconnect() {
        stopScan();
        ready = false;
        rxCharacteristic = null;
        txCharacteristic = null;
        writeQueue.clear();
        writeInProgress = false;

        if (gatt != null) {
            try {
                gatt.disconnect();
                gatt.close();
            } catch (Exception ignored) {
                // Closing an already closed GATT is harmless here.
            }
            gatt = null;
        }
    }

    public void sendLine(String line) {
        if (!ready || rxCharacteristic == null || gatt == null) {
            listener.onBleError("Inte ansluten till G30-NAV.");
            return;
        }

        String normalized = line.endsWith("\n") ? line : line + "\n";
        byte[] data = normalized.getBytes(StandardCharsets.UTF_8);
        if (data.length > 180) {
            listener.onBleError("Kommandot är för långt för BLE-paketet.");
            return;
        }

        synchronized (writeQueue) {
            writeQueue.add(data);
        }
        writeNext();
    }

    @SuppressLint("MissingPermission")
    private void writeNext() {
        if (writeInProgress || !ready || gatt == null || rxCharacteristic == null) {
            return;
        }

        final byte[] data;
        synchronized (writeQueue) {
            data = writeQueue.poll();
        }
        if (data == null) return;

        writeInProgress = true;
        boolean started;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            int result = gatt.writeCharacteristic(
                    rxCharacteristic,
                    data,
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            );
            started = result == 0;
        } else {
            rxCharacteristic.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
            rxCharacteristic.setValue(data);
            started = gatt.writeCharacteristic(rxCharacteristic);
        }

        if (!started) {
            writeInProgress = false;
            listener.onBleError("Kunde inte skicka Bluetooth-kommandot.");
            mainHandler.postDelayed(this::writeNext, 100L);
        }
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @Override
        @SuppressLint("MissingPermission")
        public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice device = result.getDevice();
            String name = null;
            try {
                name = device.getName();
            } catch (SecurityException ignored) {
                // Permission is checked before scanning.
            }

            if (DEVICE_NAME.equals(name)) {
                connect(device);
            }
        }

        @Override
        public void onScanFailed(int errorCode) {
            stopScan();
            listener.onBleError(String.format(Locale.US,
                    "BLE-skanning misslyckades (%d).", errorCode));
        }
    };

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        @SuppressLint("MissingPermission")
        public void onConnectionStateChange(BluetoothGatt callbackGatt,
                                            int status,
                                            int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                listener.onConnectionChanged(false, "Bluetooth ansluten, läser tjänster …");
                callbackGatt.requestMtu(185);
                callbackGatt.discoverServices();
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                ready = false;
                rxCharacteristic = null;
                txCharacteristic = null;
                writeInProgress = false;
                synchronized (writeQueue) {
                    writeQueue.clear();
                }
                listener.onConnectionChanged(false, "Frånkopplad");
                try {
                    callbackGatt.close();
                } catch (Exception ignored) {
                }
                if (gatt == callbackGatt) gatt = null;
            }
        }

        @Override
        @SuppressLint("MissingPermission")
        public void onServicesDiscovered(BluetoothGatt callbackGatt, int status) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                listener.onBleError("Kunde inte läsa BLE-tjänsterna.");
                return;
            }

            BluetoothGattService service = callbackGatt.getService(SERVICE_UUID);
            if (service == null) {
                listener.onBleError("G30-NAV-tjänsten saknas i ESP32-firmwaren.");
                return;
            }

            rxCharacteristic = service.getCharacteristic(RX_UUID);
            txCharacteristic = service.getCharacteristic(TX_UUID);
            if (rxCharacteristic == null || txCharacteristic == null) {
                listener.onBleError("G30-NAV RX/TX-karaktäristiken saknas.");
                return;
            }

            callbackGatt.setCharacteristicNotification(txCharacteristic, true);
            BluetoothGattDescriptor cccd = txCharacteristic.getDescriptor(CCCD_UUID);
            if (cccd != null) {
                if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                    callbackGatt.writeDescriptor(
                            cccd,
                            BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    );
                } else {
                    cccd.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
                    callbackGatt.writeDescriptor(cccd);
                }
            } else {
                finishReady();
            }
        }

        @Override
        public void onDescriptorWrite(BluetoothGatt callbackGatt,
                                      BluetoothGattDescriptor descriptor,
                                      int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                finishReady();
            } else {
                listener.onBleError("Kunde inte aktivera BLE-notiser.");
            }
        }

        @Override
        public void onCharacteristicWrite(BluetoothGatt callbackGatt,
                                          BluetoothGattCharacteristic characteristic,
                                          int status) {
            writeInProgress = false;
            if (status != BluetoothGatt.GATT_SUCCESS) {
                listener.onBleError("Ett Bluetooth-paket kunde inte skickas.");
            }
            mainHandler.postDelayed(G30BleClient.this::writeNext, 35L);
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt callbackGatt,
                                            BluetoothGattCharacteristic characteristic) {
            byte[] value = characteristic.getValue();
            deliver(value);
        }

        @Override
        public void onCharacteristicChanged(BluetoothGatt callbackGatt,
                                            BluetoothGattCharacteristic characteristic,
                                            byte[] value) {
            deliver(value);
        }
    };

    private void finishReady() {
        ready = true;
        listener.onConnectionChanged(true, "Ansluten till G30-NAV");
        mainHandler.postDelayed(() -> sendLine("CFG|GET"), 120L);
    }

    private void deliver(byte[] value) {
        if (value == null || value.length == 0) return;
        String text = new String(value, StandardCharsets.UTF_8).trim();
        if (text.isEmpty()) return;

        String[] lines = text.split("\\r?\\n");
        for (String line : lines) {
            String trimmed = line.trim();
            if (!trimmed.isEmpty()) listener.onLineReceived(trimmed);
        }
    }
}
