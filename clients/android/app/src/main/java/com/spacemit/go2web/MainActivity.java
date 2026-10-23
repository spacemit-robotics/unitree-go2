package com.spacemit.go2web;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.graphics.Color;
import android.net.ConnectivityManager;
import android.net.LinkProperties;
import android.net.Network;
import android.net.NetworkCapabilities;
import android.net.NetworkRequest;
import android.net.Uri;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;
import android.webkit.WebResourceError;
import android.webkit.WebResourceRequest;
import android.webkit.WebResourceResponse;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Button;
import android.widget.EditText;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.TextView;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

/** Fullscreen landscape host for the unchanged Go2 web controls. */
public final class MainActivity extends Activity {
    private final Handler handler = new Handler(Looper.getMainLooper());
    private final ExecutorService worker = Executors.newSingleThreadExecutor();
    private ConnectivityManager connectivity;
    private WifiManager.MulticastLock multicast;
    private Network wifi;
    private WebView web;
    private LinearLayout waiting;
    private TextView message;
    private boolean resumed;
    private boolean busy;
    private boolean pageLoaded;
    private boolean pageFailed;
    private int generation;
    private String loadedUrl = "";
    private String lastUrl = "";
    private String configuredUrl;

    private final ConnectivityManager.NetworkCallback networkCallback = new ConnectivityManager.NetworkCallback() {
        @Override public void onAvailable(Network network) { handler.post(() -> useNetwork(network)); }
        @Override public void onLost(Network network) {
            handler.post(() -> {
                if (!network.equals(wifi)) return;
                wifi = null; generation++; busy = false; lastUrl = "";
                connectivity.bindProcessToNetwork(null);
                clearPage();
                showWaiting("Wi-Fi 已断开，请连接机器狗所在的 Wi-Fi 或机器狗热点");
            });
        }
    };

    private final Runnable reconnect = new Runnable() {
        @Override public void run() {
            if (resumed && wifi != null && !busy) connect();
            if (resumed) handler.postDelayed(this, 3000);
        }
    };

    @SuppressLint("SetJavaScriptEnabled")
    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        connectivity = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        WifiManager manager = (WifiManager) getApplicationContext().getSystemService(Context.WIFI_SERVICE);
        multicast = manager.createMulticastLock("go2-web-discovery");
        multicast.setReferenceCounted(false);
        configuredUrl = getPreferences(MODE_PRIVATE).getString("url", LocalDiscovery.DEFAULT_URL);

        FrameLayout root = new FrameLayout(this);
        root.setBackgroundColor(Color.rgb(7, 17, 22));
        web = new WebView(this);
        web.setBackgroundColor(Color.rgb(7, 17, 22));
        WebSettings settings = web.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(true);
        settings.setCacheMode(WebSettings.LOAD_NO_CACHE);
        settings.setAllowFileAccess(false);
        settings.setAllowContentAccess(false);
        settings.setMediaPlaybackRequiresUserGesture(false);
        web.setWebViewClient(new WebViewClient() {
            @Override public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
                if (!request.isForMainFrame()) return false;
                Uri current = Uri.parse(loadedUrl), destination = request.getUrl();
                return !"http".equals(destination.getScheme()) || !java.util.Objects.equals(current.getHost(), destination.getHost())
                        || current.getPort() != destination.getPort();
            }
            @Override public void onPageStarted(WebView view, String url, android.graphics.Bitmap icon) {
                if (url.equals(loadedUrl)) pageFailed = false;
            }
            @Override public void onPageFinished(WebView view, String url) {
                if (!pageFailed && url.equals(loadedUrl) && resumed) {
                    pageLoaded = true;
                    waiting.setVisibility(View.GONE);
                    immersive();
                }
            }
            @Override public void onReceivedError(WebView view, WebResourceRequest request, WebResourceError error) {
                if (request.isForMainFrame()) failPage();
            }
            @Override public void onReceivedHttpError(WebView view, WebResourceRequest request, WebResourceResponse response) {
                if (request.isForMainFrame() && response.getStatusCode() >= 400) failPage();
            }
        });
        root.addView(web, new FrameLayout.LayoutParams(-1, -1));

        waiting = new LinearLayout(this);
        waiting.setOrientation(LinearLayout.VERTICAL);
        waiting.setGravity(Gravity.CENTER);
        waiting.setPadding(32, 24, 32, 24);
        waiting.setBackgroundColor(Color.rgb(7, 17, 22));
        TextView title = new TextView(this);
        title.setText("Go2 网页控制"); title.setTextSize(26); title.setTextColor(Color.rgb(58, 219, 203));
        waiting.addView(title);
        message = new TextView(this);
        message.setTextColor(Color.LTGRAY); message.setTextSize(16); message.setGravity(Gravity.CENTER);
        message.setPadding(0, 20, 0, 20);
        waiting.addView(message);
        LinearLayout buttons = new LinearLayout(this);
        Button retry = new Button(this); retry.setText("重新连接"); retry.setOnClickListener(v -> retry());
        Button address = new Button(this); address.setText("设置地址"); address.setOnClickListener(v -> addressDialog());
        buttons.addView(retry); buttons.addView(address); waiting.addView(buttons);
        root.addView(waiting, new FrameLayout.LayoutParams(-1, -1));
        setContentView(root);
        showWaiting("请连接机器狗所在的 Wi-Fi 或机器狗热点");
        NetworkRequest request = new NetworkRequest.Builder().addTransportType(NetworkCapabilities.TRANSPORT_WIFI)
                .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET).build();
        connectivity.registerNetworkCallback(request, networkCallback);
    }

    private void useNetwork(Network network) {
        if (isFinishing() || isDestroyed() || network.equals(wifi)) return;
        generation++; busy = false; lastUrl = ""; wifi = network;
        connectivity.bindProcessToNetwork(network);
        clearPage();
        showWaiting("正在寻找机器狗网页…");
        if (resumed) connect();
    }

    private void connect() {
        if (!resumed || wifi == null || busy) return;
        busy = true;
        final int attempt = generation;
        final Network network = wifi;
        final LinkProperties links = connectivity.getLinkProperties(network);
        final String config = configuredUrl, previous = lastUrl, current = loadedUrl;
        final boolean loaded = pageLoaded;
        if (!loaded) showWaiting("正在寻找机器狗网页…");
        worker.execute(() -> {
            String found;
            if (loaded && LocalDiscovery.isServer(network, current)) found = current;
            else found = LocalDiscovery.find(network, links, config, previous);
            final String result = found;
            handler.post(() -> {
                if (attempt != generation || !network.equals(wifi) || !resumed) return;
                busy = false;
                if (result == null) {
                    clearPage();
                    showWaiting("未找到网页服务\n请确认机器狗已运行程序，并连接同一 Wi-Fi 或机器狗热点");
                } else if (!pageLoaded || !result.equals(loadedUrl)) {
                    clearPage();
                    lastUrl = result; loadedUrl = result; pageFailed = false;
                    web.loadUrl(result);
                }
            });
        });
    }

    private void failPage() {
        pageFailed = true; pageLoaded = false;
        showWaiting("网页连接中断，正在重连…");
    }

    private void clearPage() {
        pageLoaded = false; loadedUrl = "";
        web.stopLoading();
        web.loadUrl("about:blank");
    }

    private void showWaiting(String text) {
        message.setText(text); waiting.setVisibility(View.VISIBLE);
    }

    private void retry() {
        generation++; busy = false;
        clearPage();
        if (wifi == null) showWaiting("请连接机器狗所在的 Wi-Fi 或机器狗热点");
        else connect();
    }

    private void addressDialog() {
        EditText field = new EditText(this);
        field.setSingleLine(true); field.setText(configuredUrl);
        field.setInputType(android.text.InputType.TYPE_CLASS_TEXT | android.text.InputType.TYPE_TEXT_VARIATION_URI);
        new AlertDialog.Builder(this).setTitle("网页地址或 IP").setView(field)
                .setNeutralButton("自动寻找", (dialog, which) -> saveAddress(LocalDiscovery.DEFAULT_URL))
                .setNegativeButton("取消", null).setPositiveButton("连接", (dialog, which) -> {
                    String value = field.getText().toString().trim();
                    if (!value.contains("://")) value = "http://" + value;
                    Uri uri = Uri.parse(value);
                    if (!"http".equals(uri.getScheme()) || uri.getHost() == null || uri.getUserInfo() != null
                            || uri.getQuery() != null || uri.getFragment() != null
                            || (uri.getPath() != null && !uri.getPath().isEmpty() && !uri.getPath().equals("/"))) {
                        new AlertDialog.Builder(this).setMessage("请输入 HTTP 网页地址，例如 http://go2-sdk.local:8082/")
                                .setPositiveButton("确定", null).show();
                        return;
                    }
                    if (uri.getPort() < 0) value = "http://" + uri.getHost() + ":8082/";
                    else if (!value.endsWith("/")) value += "/";
                    saveAddress(value);
                }).show();
    }

    private void saveAddress(String value) {
        configuredUrl = value; lastUrl = "";
        getPreferences(MODE_PRIVATE).edit().putString("url", value).apply();
        retry();
    }

    @SuppressWarnings("deprecation")
    @Override public void onBackPressed() {
        web.evaluateJavascript("window.dispatchEvent(new Event('pagehide'));", null);
        new AlertDialog.Builder(this).setTitle("Go2 网页控制")
                .setItems(new String[]{"重新连接", "设置地址", "退出"}, (dialog, which) -> {
                    if (which == 0) retry(); else if (which == 1) addressDialog(); else finish();
                }).show();
    }

    @Override public void onResume() {
        super.onResume(); resumed = true;
        if (!multicast.isHeld()) multicast.acquire();
        web.onResume(); web.resumeTimers(); immersive();
        handler.removeCallbacks(reconnect); handler.post(reconnect);
    }

    @Override public void onPause() {
        web.evaluateJavascript("window.dispatchEvent(new Event('pagehide'));", null);
        resumed = false; generation++; busy = false;
        handler.removeCallbacks(reconnect);
        web.onPause(); web.pauseTimers();
        if (multicast.isHeld()) multicast.release();
        super.onPause();
    }

    @Override public void onDestroy() {
        connectivity.unregisterNetworkCallback(networkCallback);
        connectivity.bindProcessToNetwork(null);
        generation++; worker.shutdownNow(); handler.removeCallbacksAndMessages(null);
        if (multicast.isHeld()) multicast.release();
        web.destroy(); super.onDestroy();
    }

    @SuppressWarnings("deprecation")
    private void immersive() {
        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_STABLE);
    }

    @Override public void onWindowFocusChanged(boolean focus) {
        super.onWindowFocusChanged(focus);
        if (focus) immersive();
    }
}
