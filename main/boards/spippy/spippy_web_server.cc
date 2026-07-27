#include "spippy_web_server.h"

#include <cstdlib>
#include <cstring>
#include <cmath>

#include <esp_check.h>
#include <esp_log.h>
#include <mdns.h>

#include "spippy_controller.h"

static const char *TAG = "SpippyWeb";
static constexpr size_t kMaxRequestBodyBytes = 1024;

static bool JsonIntegerInRange(const cJSON *value, int min_value, int max_value, int *result)
{
    if (!cJSON_IsNumber(value) || result == nullptr || !std::isfinite(value->valuedouble) ||
        std::floor(value->valuedouble) != value->valuedouble ||
        value->valuedouble < min_value || value->valuedouble > max_value) {
        return false;
    }
    *result = static_cast<int>(value->valuedouble);
    return true;
}

static bool JsonScaledNumberInRange(const cJSON *value,
                                    double scale,
                                    int min_value,
                                    int max_value,
                                    int *result)
{
    if (!cJSON_IsNumber(value) || result == nullptr || !std::isfinite(value->valuedouble)) {
        return false;
    }
    const double scaled = value->valuedouble * scale;
    if (!std::isfinite(scaled) || scaled < min_value || scaled > max_value) {
        return false;
    }
    *result = static_cast<int>(std::lround(scaled));
    return true;
}

static const char kCalibrationPage[] =
R"HTML(<!doctype html><html lang=zh-CN><head><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1,viewport-fit=cover">
<title>斯皮皮 Spippy 舵机校准</title><style>
:root{color-scheme:light;--bg:#f3f6f2;--ink:#16231f;--muted:#69756f;--panel:rgba(255,255,255,.94);--line:#dce5df;--green:#226f58;--green2:#e3f2ec;--blue:#2b658d;--blue2:#eaf3fa;--shadow:0 14px 36px rgba(33,61,49,.08)}
*{box-sizing:border-box}body{margin:0;min-height:100vh;background:radial-gradient(circle at 50% -10%,#dcefe6 0,transparent 38%),var(--bg);color:var(--ink);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;-webkit-font-smoothing:antialiased}.app{max-width:860px;margin:0 auto;padding:0 24px calc(36px + env(safe-area-inset-bottom))}
header{position:sticky;top:0;z-index:6;margin:0 0 20px;padding:calc(18px + env(safe-area-inset-top)) 24px 16px;background:rgba(243,246,242,.9);backdrop-filter:blur(16px);border-bottom:1px solid rgba(202,216,207,.8)}.top{display:flex;align-items:center;justify-content:space-between;gap:24px;max-width:812px;margin:0 auto}.eyebrow{display:block;margin-bottom:5px;color:var(--green);font-size:11px;font-weight:850;letter-spacing:.14em;text-transform:uppercase}h1{margin:0;font-size:24px;line-height:1.15;letter-spacing:-.02em}.sub{margin:6px 0 0;color:var(--muted);font-size:13px}.header-actions{display:flex;align-items:center;gap:8px;flex:0 0 auto}
button{appearance:none;display:inline-flex;align-items:center;justify-content:center;min-height:44px;padding:10px 15px;border:1px solid var(--line);border-radius:11px;background:#fff;color:var(--ink);font-size:14px;font-weight:800;text-align:center;cursor:pointer;transition:transform .14s ease,box-shadow .14s ease,opacity .14s ease}button:active{transform:translateY(1px)}button:disabled,input:disabled{opacity:.42;cursor:not-allowed}.primary{background:var(--green);border-color:var(--green);color:#fff;box-shadow:0 7px 18px rgba(34,111,88,.18)}.soft{background:var(--green2);border-color:#bddfcf;color:#175844}.blue{background:var(--blue2);border-color:#c9ddec;color:var(--blue)}.autonomy.on{background:var(--green2);border-color:#b8dacb;color:#175844}.autonomy.off{background:#f1f2f1;color:#727b77}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:15px;box-shadow:var(--shadow);padding:18px;margin:14px 0}.section-title{display:flex;align-items:center;justify-content:space-between;gap:12px;margin-bottom:14px}h2{font-size:17px;margin:0}.hint{font-size:12px;color:var(--muted)}.status-pill{padding:5px 9px;border-radius:999px;background:#eef2ef;color:#5e6c65;font-weight:750}.status-pill.active{background:var(--green2);color:#175844}
.bar{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:10px}.bar button{width:100%}.meta{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-top:12px}.metric{min-height:70px;border:1px solid #e2e9e4;border-radius:12px;padding:11px;background:#fbfcfb}.metric span{display:block;color:var(--muted);font-size:12px;margin-bottom:8px}.metric strong{font-size:16px;overflow-wrap:anywhere}.metric button{width:100%;min-height:32px;padding:5px 9px;font-size:13px}.preflight.running{background:#fff3e5;border-color:#edc995;color:#8a4b12}.notice{margin-top:12px;padding:10px 12px;border-radius:10px;background:#f2f6f3;color:#5f6d66;font-size:12px;line-height:1.55}
.legs{display:grid;gap:12px}.leg-row{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}.leg{border:1px solid var(--line);border-radius:13px;background:#fbfcfb;padding:12px}.leg.active{border-color:#91c9b3;background:#f4faf7}.leg-title{display:flex;justify-content:space-between;margin-bottom:10px;color:#34443d;font-size:14px;font-weight:850}.joints{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.joint{border:1px solid #e0e7e2;border-radius:11px;background:#fff;padding:10px}.joint.active{outline:2px solid var(--green);border-color:transparent}.joint-head{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-bottom:9px}.joint-name{font-size:14px;font-weight:850}.offset{color:var(--blue);font-variant-numeric:tabular-nums;font-weight:850}.stepper{display:grid;grid-template-columns:40px 1fr 40px;gap:6px}.stepper button{min-height:38px;padding:0;font-size:20px}.stepper input{width:100%;height:40px;border:1px solid var(--line);border-radius:9px;background:#fff;color:var(--ink);font-size:15px;font-weight:800;text-align:center}
.editor{display:grid;gap:13px}.selected{display:flex;align-items:center;justify-content:space-between;gap:12px}.big{font-size:36px;font-weight:900;letter-spacing:-.03em}.range{width:100%;accent-color:var(--green)}.fine{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}.fine button,#zeroBtn{width:100%}.toast{position:fixed;left:14px;right:14px;bottom:calc(16px + env(safe-area-inset-bottom));z-index:20;display:none;max-width:540px;margin:0 auto;padding:13px 15px;border-radius:11px;background:#17231f;color:#fff;font-size:14px;box-shadow:0 16px 34px rgba(0,0,0,.2)}
@media(max-width:720px){.app{padding:0 14px calc(28px + env(safe-area-inset-bottom))}header{padding-left:14px;padding-right:14px}.top{align-items:flex-start;gap:12px}.header-actions{flex-direction:column;align-items:stretch}.header-actions button{min-height:40px;padding:8px 10px;font-size:12px}.meta{grid-template-columns:repeat(2,minmax(0,1fr))}.bar{grid-template-columns:1fr}.leg-row{gap:8px}.joints{grid-template-columns:1fr}.panel{padding:14px;border-radius:13px}.fine{grid-template-columns:repeat(2,1fr)}}
</style></head><body><header><div class=top><div><span class=eyebrow>Spippy Care</span><h1>斯皮皮 · 舵机校准</h1><p class=sub>安全进入 90° 基准位，再逐个微调腿部偏移</p></div><div class=header-actions><button id=autonomyBtn class=autonomy>自主互动 · --</button></div></div></header>
<main class=app><section class=panel><div class=section-title><h2>校准会话</h2><span class="hint status-pill" id=sessionHint>正在连接</span></div><div class=bar><button id=startBtn class=primary>开始校准</button><button id=saveExitBtn class=soft disabled>保存并完成</button></div><div class=meta><div class=metric><span>当前姿态</span><strong data-k=pose>--</strong></div><div class=metric><span>电池状态</span><strong data-k=power>--</strong></div><div class=metric><span>当前腿位</span><strong data-k=activeLeg>--</strong></div><div class=metric><span>舵机预检</span><button id=preflightBtn class=blue>开始预检</button></div></div><div class=notice id=connectionNote>预检与校准相互独立，运行其中一项时另一项不可操作。</div></section>
<section class=panel><div class=section-title><h2>四腿布局</h2><span class=hint>前排 / 后排</span></div><div id=legs class=legs></div></section>
<section class=panel><div class=section-title><h2>精细调整</h2><span class=hint>目标角度 = 90° + 偏移</span></div><div class=editor><div class=selected><div><div class=hint id=selectedLabel>请选择舵机</div><div class=big id=angleReadout>90°</div></div><button id=zeroBtn class=blue style=max-width:120px disabled>偏移归零</button></div><input id=offsetRange class=range type=range min=-45 max=45 step=1 value=0 disabled><div class=fine><button data-nudge=-5 disabled>-5°</button><button data-nudge=-1 disabled>-1°</button><button data-nudge=1 disabled>+1°</button><button data-nudge=5 disabled>+5°</button></div></div></section></main><div id=toast class=toast></div>
<script>
let status=null,channels=[],selected=0,refreshing=false,preflightBusy=false,updateChain=Promise.resolve();const API='/spippy/api',legNames={0:'左前腿',1:'右前腿',2:'左后腿',3:'右后腿'},jointNames={0:'根部',1:'膝部'},legLayout=[[0,1],[2,3]],$=id=>document.getElementById(id),setText=(k,v)=>{const e=document.querySelector(`[data-k="${k}"]`);if(e)e.textContent=v};
function toast(msg){const e=$('toast');e.textContent=msg;e.style.display='block';clearTimeout(window.__toast);window.__toast=setTimeout(()=>e.style.display='none',2400)}function errText(e){return e&&e.message?e.message:'操作失败'}
async function api(path,body,retry=true){const method=body===undefined?'GET':'POST',controller=new AbortController(),timer=setTimeout(()=>controller.abort(),6000);try{const r=await fetch(API+path,{method,headers:{'content-type':'application/json'},body:body===undefined?undefined:JSON.stringify(body),cache:'no-store',signal:controller.signal});const j=await r.json().catch(()=>({}));if(!r.ok||j.status==='error')throw Error(j.message||j.error||j.code||('HTTP '+r.status));return j}catch(e){if(retry&&method==='GET'){await new Promise(resolve=>setTimeout(resolve,350));return api(path,body,false)}throw e}finally{clearTimeout(timer)}}
function offsetOf(ch){return Number(ch.zero_offset!==undefined?ch.zero_offset:(ch.zero_offset_deg_x10||0)/10)}function channelName(ch){return `${legNames[ch.leg_id]||('腿'+ch.leg_id)} ${jointNames[ch.joint_id]||('关节'+ch.joint_id)}`}function current(){return channels.find(ch=>Number(ch.servo_index??ch.index)===selected)||channels[0]}function currentLeg(){const ch=current();return ch?Number(ch.leg_id):0}function grouped(){const g={};channels.forEach(ch=>{(g[ch.leg_id]||(g[ch.leg_id]=[])).push(ch)});return Object.keys(g).sort((a,b)=>a-b).map(k=>({leg:Number(k),items:g[k].sort((a,b)=>Number(a.joint_id)-Number(b.joint_id))}))}
function renderLegs(){const root=$('legs');root.innerHTML='';const active=!!(status&&status.preview_mode_enabled),groups=grouped();legLayout.forEach(rowLegs=>{const row=document.createElement('div');row.className='leg-row';rowLegs.forEach(legId=>{const group=groups.find(x=>x.leg===legId);if(!group)return;const leg=document.createElement('div');leg.className='leg'+(group.leg===currentLeg()?' active':'');leg.innerHTML=`<div class=leg-title><span>${legNames[group.leg]||('腿 '+group.leg)}</span><span>${group.items.length} 个舵机</span></div>`;const joints=document.createElement('div');joints.className='joints';group.items.forEach(ch=>{const idx=Number(ch.servo_index??ch.index),off=offsetOf(ch),card=document.createElement('div');card.className='joint'+(idx===selected?' active':'');card.innerHTML=`<div class=joint-head><span class=joint-name>${jointNames[ch.joint_id]||ch.label}</span><span class=offset>${off.toFixed(0)}°</span></div><div class=stepper><button data-dec="${idx}" ${active?'':'disabled'}>-</button><input data-input="${idx}" value="${off.toFixed(0)}" ${active?'':'disabled'}><button data-inc="${idx}" ${active?'':'disabled'}>+</button></div><div class=hint style=margin-top:7px>GPIO ${ch.gpio} · #${idx}</div>`;card.onclick=e=>{if(e.target.tagName!=='BUTTON'&&e.target.tagName!=='INPUT'){selected=idx;render()}};joints.appendChild(card)});leg.appendChild(joints);row.appendChild(leg)});root.appendChild(row)});document.querySelectorAll('[data-dec]').forEach(b=>b.onclick=e=>{e.stopPropagation();nudge(Number(b.dataset.dec),-1)});document.querySelectorAll('[data-inc]').forEach(b=>b.onclick=e=>{e.stopPropagation();nudge(Number(b.dataset.inc),1)});document.querySelectorAll('[data-input]').forEach(i=>i.onchange=e=>{e.stopPropagation();setOffset(Number(i.dataset.input),Number(i.value))})}
function render(rebuildLegs=true){if(!status)return;const active=!!status.preview_mode_enabled,preflight=!!status.preflight_running,ch=current(),autonomy=!!status.autonomy_enabled,p=status.power||status,lowPower=!!p.low_power_latched;if(rebuildLegs)renderLegs();$('sessionHint').textContent=preflight?'舵机预检中':(active?'90° 基准已开启':'未进入校准');$('sessionHint').className='hint status-pill'+((active||preflight)?' active':'');$('startBtn').textContent=active?'取消本次校准':'开始校准';$('startBtn').disabled=preflight||preflightBusy;$('saveExitBtn').disabled=!active;$('preflightBtn').textContent=preflight?'停止预检':'开始预检';$('preflightBtn').disabled=preflightBusy||active||(lowPower&&!preflight);$('preflightBtn').className=preflight?'preflight running':'blue';setText('pose',status.pose_name||status.pose||'--');setText('power',lowPower?'低电保护':(((p.filtered_battery_voltage_mv||p.battery_voltage_mv||0)/1000).toFixed(2))+' V');setText('activeLeg',legNames[currentLeg()]||'--');document.querySelectorAll('[data-nudge]').forEach(e=>e.disabled=!active);$('offsetRange').disabled=!active;$('zeroBtn').disabled=!active;$('autonomyBtn').textContent='自主互动 · '+(autonomy?'开':'关');$('autonomyBtn').className='autonomy '+(autonomy?'on':'off');$('autonomyBtn').setAttribute('aria-pressed',String(autonomy));$('connectionNote').textContent=preflight?'预检运行时校准和舵机偏移调节已锁定；停止后机器人恢复正常待机。':(active?'校准中不可启动预检；取消会撤销本次未保存的修改。':'预检可直接启动；停止预检后恢复正常待机。需要 90° 归中时请单独开始校准。');if(ch){$('selectedLabel').textContent=`${channelName(ch)}  #${selected}`;$('angleReadout').textContent=`${(90+offsetOf(ch)).toFixed(0)}°`;$('offsetRange').value=String(offsetOf(ch))}}
async function refresh(quiet=false){if(refreshing)return;refreshing=true;try{const next=await api('/calibration');status=next;channels=next.channels||[];channels.forEach(ch=>{ch.servo_index=Number(ch.servo_index??ch.index);ch.zero_offset=offsetOf(ch)});if(!channels.find(ch=>ch.servo_index===selected)&&channels[0])selected=channels[0].servo_index;render()}catch(e){$('connectionNote').textContent='连接暂时中断，请确认设备和当前终端位于同一 Wi-Fi。';if(!quiet)toast(errText(e))}finally{refreshing=false}}
function setOffset(idx,value){const ch=channels.find(x=>x.servo_index===idx);if(!ch||!(status&&status.preview_mode_enabled))return;selected=idx;const normalized=Math.max(-45,Math.min(45,Number(value)||0));ch.zero_offset=normalized;render();updateChain=updateChain.catch(()=>{}).then(()=>api('/calibration/servo',{servo_index:idx,zero_offset:normalized},false));updateChain.then(()=>toast('偏移已预览')).catch(async e=>{toast(errText(e));await refresh(true)})}function nudge(idx,delta){const ch=channels.find(x=>x.servo_index===idx);if(ch)setOffset(idx,offsetOf(ch)+delta)}
async function toggleSession(){const active=!!(status&&status.preview_mode_enabled);await api('/calibration/session',{enabled:!active},false);await refresh(true);toast(active?'已取消，未保存修改已撤销':'已进入安全校准模式')}async function togglePreflight(){if(!status||preflightBusy)return;const previous=!!status.preflight_running,previousPose=status.pose_name,desired=!previous;preflightBusy=true;status.preflight_running=desired;status.pose_name=desired?'safe':'stand';render(false);try{await api('/calibration/preflight',{enabled:desired},false);toast(desired?'舵机预检已开始':'预检已停止，机器人恢复正常待机')}catch(e){status.preflight_running=previous;status.pose_name=previousPose;toast(errText(e))}finally{preflightBusy=false;render(false)}}async function saveSession(){await updateChain;await api('/calibration/save',{},false);await refresh(true);toast('校准数据已保存')}async function toggleAutonomy(){const desired=!(status&&status.autonomy_enabled);$('autonomyBtn').disabled=true;try{await api('/autonomy',{enabled:desired},false);status.autonomy_enabled=desired;render(false);toast(desired?'自主互动已开启':'自主互动已关闭')}finally{$('autonomyBtn').disabled=false}}
$('startBtn').onclick=()=>toggleSession().catch(e=>toast(errText(e)));$('preflightBtn').onclick=()=>togglePreflight();$('saveExitBtn').onclick=()=>saveSession().catch(e=>toast(errText(e)));$('autonomyBtn').onclick=()=>toggleAutonomy().catch(e=>toast(errText(e)));$('offsetRange').oninput=e=>{const ch=current();if(ch){ch.zero_offset=Number(e.target.value);$('angleReadout').textContent=`${(90+ch.zero_offset).toFixed(0)}°`}};$('offsetRange').onchange=()=>{const ch=current();if(ch)setOffset(ch.servo_index,offsetOf(ch))};document.querySelectorAll('[data-nudge]').forEach(b=>b.onclick=()=>{const ch=current();if(ch)setOffset(ch.servo_index,offsetOf(ch)+Number(b.dataset.nudge))});$('zeroBtn').onclick=()=>{const ch=current();if(ch)setOffset(ch.servo_index,0)};refresh(false);setInterval(()=>{if(!(status&&(status.preview_mode_enabled||status.preflight_running)))refresh(true)},15000);
</script></body></html>)HTML";

static cJSON *MakeErrorJson(const char *code, const char *message)
{
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "status", "error");
    cJSON_AddStringToObject(root, "code", code);
    cJSON_AddStringToObject(root, "message", message);
    return root;
}

SpippyWebServer::SpippyWebServer(SpippyController *controller) : controller_(controller) {
}

SpippyWebServer::~SpippyWebServer() {
    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }
}

bool SpippyWebServer::Start() {
    if (server_ != nullptr) {
        return true;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 16;
    if (httpd_start(&server_, &config) != ESP_OK) {
        ESP_LOGE(TAG, "failed to start");
        return false;
    }
    httpd_uri_t routes[] = {
        {.uri = "/", .method = HTTP_GET, .handler = HandleRoot, .user_ctx = this},
        {.uri = "/spippy", .method = HTTP_GET, .handler = HandlePage, .user_ctx = this},
        {.uri = "/spippy/api/status", .method = HTTP_GET, .handler = HandleStatus, .user_ctx = this},
        {.uri = "/spippy/api/calibration", .method = HTTP_GET, .handler = HandleCalibration, .user_ctx = this},
        {.uri = "/spippy/api/calibration/session", .method = HTTP_POST, .handler = HandlePreview, .user_ctx = this},
        {.uri = "/spippy/api/calibration/preflight", .method = HTTP_POST, .handler = HandlePreflight, .user_ctx = this},
        {.uri = "/spippy/api/calibration/preview", .method = HTTP_POST, .handler = HandleCalibrationPreviewServo, .user_ctx = this},
        {.uri = "/spippy/api/calibration/servo", .method = HTTP_POST, .handler = HandleCalibrationServoUpdate, .user_ctx = this},
        {.uri = "/spippy/api/pose", .method = HTTP_POST, .handler = HandlePose, .user_ctx = this},
        {.uri = "/spippy/api/save", .method = HTTP_POST, .handler = HandleSave, .user_ctx = this},
        {.uri = "/spippy/api/calibration/save", .method = HTTP_POST, .handler = HandleSave, .user_ctx = this},
        {.uri = "/spippy/api/autonomy", .method = HTTP_POST, .handler = HandleAutonomy, .user_ctx = this},
    };
    for (auto &route : routes) {
        esp_err_t err = httpd_register_uri_handler(server_, &route);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "failed to register %s: %s", route.uri, esp_err_to_name(err));
            httpd_stop(server_);
            server_ = nullptr;
            return false;
        }
    }
    esp_err_t mdns_err = StartMdns();
    if (mdns_err != ESP_OK) {
        ESP_LOGW(TAG, "web server is available by IP but mDNS is unavailable: %s", esp_err_to_name(mdns_err));
    }
    ESP_LOGI(TAG, "started on http://spippy.local/spippy");
    return true;
}

esp_err_t SpippyWebServer::StartMdns() {
    esp_err_t err = mdns_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_RETURN_ON_ERROR(mdns_hostname_set("spippy"), TAG, "failed to set mDNS hostname");
    ESP_RETURN_ON_ERROR(mdns_instance_name_set("Spippy (斯皮皮) Calibration"), TAG, "failed to set mDNS instance");

    mdns_txt_item_t service_txt[] = {
        {"path", "/spippy"},
    };
    err = mdns_service_add("Spippy (斯皮皮) Calibration", "_http", "_tcp", 80, service_txt,
                           sizeof(service_txt) / sizeof(service_txt[0]));
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "mDNS service add failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t SpippyWebServer::HandlePage(httpd_req_t *req) {
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    return httpd_resp_send(req, kCalibrationPage, HTTPD_RESP_USE_STRLEN);
}

esp_err_t SpippyWebServer::HandleRoot(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/spippy");
    return httpd_resp_send(req, nullptr, 0);
}

esp_err_t SpippyWebServer::SendJson(httpd_req_t *req, cJSON *json) {
    if (json == nullptr) {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json unavailable");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    char *text = cJSON_PrintUnformatted(json);
    if (text == nullptr) {
        cJSON_Delete(json);
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "json print failed");
    }
    esp_err_t err = httpd_resp_sendstr(req, text);
    cJSON_free(text);
    cJSON_Delete(json);
    return err;
}

char *SpippyWebServer::ReadBody(httpd_req_t *req) {
    if (req->content_len > kMaxRequestBodyBytes) {
        return nullptr;
    }
    char *buf = static_cast<char *>(calloc(req->content_len + 1, 1));
    if (buf == nullptr) {
        return nullptr;
    }
    int received = 0;
    int timeout_retries = 0;
    while (received < req->content_len) {
        int ret = httpd_req_recv(req, buf + received, req->content_len - received);
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            if (++timeout_retries > 2) {
                free(buf);
                return nullptr;
            }
            continue;
        }
        if (ret <= 0) {
            free(buf);
            return nullptr;
        }
        received += ret;
    }
    return buf;
}

esp_err_t SpippyWebServer::HandleStatus(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    return self->SendJson(req, self->controller_->StatusJson());
}

esp_err_t SpippyWebServer::HandleCalibration(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    return self->SendJson(req, self->controller_->CalibrationStatusJson());
}

esp_err_t SpippyWebServer::HandlePreview(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    char *body = self->ReadBody(req);
    cJSON *json = body ? cJSON_Parse(body) : nullptr;
    cJSON *enabled_value = cJSON_GetObjectItem(json, "enabled");
    if (!cJSON_IsBool(enabled_value)) {
        free(body);
        cJSON_Delete(json);
        return self->SendJson(req, MakeErrorJson("invalid_enabled", "enabled must be true or false"));
    }
    bool enabled = cJSON_IsTrue(enabled_value);
    free(body);
    cJSON_Delete(json);
    return self->SendJson(req, self->controller_->CalibrationPreview(enabled));
}

esp_err_t SpippyWebServer::HandlePreflight(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    char *body = self->ReadBody(req);
    cJSON *json = body ? cJSON_Parse(body) : nullptr;
    cJSON *enabled_value = cJSON_GetObjectItem(json, "enabled");
    if (!cJSON_IsBool(enabled_value)) {
        free(body);
        cJSON_Delete(json);
        return self->SendJson(req, MakeErrorJson("invalid_preflight_enabled", "enabled must be true or false"));
    }
    bool enabled = cJSON_IsTrue(enabled_value);
    free(body);
    cJSON_Delete(json);
    return self->SendJson(req, self->controller_->CalibrationPreflight(enabled));
}

esp_err_t SpippyWebServer::HandleCalibrationPreviewServo(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    char *body = self->ReadBody(req);
    cJSON *json = body ? cJSON_Parse(body) : nullptr;
    cJSON *index = cJSON_GetObjectItem(json, "servo_index");
    if (!cJSON_IsNumber(index)) {
        index = cJSON_GetObjectItem(json, "index");
    }
    cJSON *angle = cJSON_GetObjectItem(json, "angle_deg");
    cJSON *result = nullptr;
    int servo_index = 0;
    int angle_x10 = 0;
    if (JsonIntegerInRange(index, 0, SPIPPY_ACTIVE_SERVO_COUNT - 1, &servo_index) &&
        JsonScaledNumberInRange(angle, 10.0, -450, 450, &angle_x10)) {
        result = self->controller_->CalibrationServo(servo_index, angle_x10);
    } else {
        result = MakeErrorJson("invalid_preview_request", "servo_index and angle_deg are required numbers");
    }
    free(body);
    cJSON_Delete(json);
    return self->SendJson(req, result);
}

esp_err_t SpippyWebServer::HandleCalibrationServoUpdate(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    char *body = self->ReadBody(req);
    cJSON *json = body ? cJSON_Parse(body) : nullptr;
    cJSON *index = cJSON_GetObjectItem(json, "servo_index");
    if (!cJSON_IsNumber(index)) {
        index = cJSON_GetObjectItem(json, "index");
    }
    cJSON *offset = cJSON_GetObjectItem(json, "zero_offset_deg_x10");
    int servo_index = 0;
    int offset_x10 = 0;
    bool valid = JsonIntegerInRange(index, 0, SPIPPY_ACTIVE_SERVO_COUNT - 1, &servo_index);
    if (cJSON_IsNumber(offset)) {
        valid = valid && JsonScaledNumberInRange(offset, 1.0, -450, 450, &offset_x10);
    } else {
        offset = cJSON_GetObjectItem(json, "zero_offset");
        valid = valid && JsonScaledNumberInRange(offset, 10.0, -450, 450, &offset_x10);
    }
    cJSON *result = valid
        ? self->controller_->CalibrationServoUpdate(servo_index, offset_x10)
        : MakeErrorJson("invalid_servo_update", "servo_index and zero_offset are required numbers");
    free(body);
    cJSON_Delete(json);
    return self->SendJson(req, result);
}

esp_err_t SpippyWebServer::HandlePose(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    char *body = self->ReadBody(req);
    cJSON *json = body ? cJSON_Parse(body) : nullptr;
    cJSON *pose = cJSON_GetObjectItem(json, "pose");
    cJSON *result = cJSON_IsString(pose) ? self->controller_->CalibrationPose(pose->valuestring)
                                         : MakeErrorJson("invalid_pose_request", "pose is required");
    free(body);
    cJSON_Delete(json);
    return self->SendJson(req, result);
}

esp_err_t SpippyWebServer::HandleSave(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    return self->SendJson(req, self->controller_->CalibrationSave());
}

esp_err_t SpippyWebServer::HandleAutonomy(httpd_req_t *req) {
    auto *self = static_cast<SpippyWebServer *>(req->user_ctx);
    char *body = self->ReadBody(req);
    cJSON *json = body ? cJSON_Parse(body) : nullptr;
    cJSON *enabled_value = cJSON_GetObjectItem(json, "enabled");
    cJSON *result = cJSON_IsBool(enabled_value)
        ? self->controller_->SetAutonomyEnabled(cJSON_IsTrue(enabled_value))
        : MakeErrorJson("invalid_autonomy_enabled", "enabled must be true or false");
    free(body);
    cJSON_Delete(json);
    return self->SendJson(req, result);
}
