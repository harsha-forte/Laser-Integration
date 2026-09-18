"""
laser_app_console.py - alternative cockpit UI for laser marking automation
Run: sudo venv/bin/python3 laser_app_console.py
Open: http://192.168.17.132:5000
"""

from flask import Flask, jsonify, request, render_template_string
from laser_ctrl import (
    controller, State, MAX_TRAVEL_MM,
    PART_HEIGHT_MIN_MM, PART_HEIGHT_MAX_MM,
)
import threading, atexit

app = Flask(__name__)

threading.Thread(target=controller.startup, daemon=True).start()
atexit.register(controller.shutdown)

HTML = """<!DOCTYPE html>
<html lang="en" data-theme="{{ theme }}">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Laser Cell Console</title>
<style>
*{box-sizing:border-box}html,body{margin:0;width:100%;height:100%}button,input,select{font:inherit}
[data-theme="dark"]{
  --bg:#060a0c;--panel:#0d1417;--panel2:#111c20;--panel3:#17262a;--line:#22353a;
  --text:#eaf7f4;--muted:#7c9691;--soft:#9fb5b1;--input:#091013;--shadow:rgba(0,0,0,.45);
}
[data-theme="light"]{
  --bg:#edf3f1;--panel:#fbfdfc;--panel2:#f1f6f4;--panel3:#e5efec;--line:#cbdad6;
  --text:#10201d;--muted:#607873;--soft:#405a55;--input:#ffffff;--shadow:rgba(17,42,36,.12);
}
:root{--accent:#00d7a3;--accent2:#00a8ff;--warn:#ffbe3d;--danger:#ff4056;--purple:#bd6cff;--header:76px;--terminal:155px}
body{background:var(--bg);color:var(--text);font-family:'Segoe UI',Arial,sans-serif;overflow:hidden;display:flex;flex-direction:column}

/* top command bar */
.topbar{height:var(--header);flex:none;background:var(--panel);border-bottom:1px solid var(--line);display:grid;grid-template-columns:310px 1fr auto;align-items:center;padding:0 18px;box-shadow:0 10px 28px var(--shadow);z-index:10}
.brand{display:flex;align-items:center;gap:13px}.brand-symbol{width:43px;height:43px;border:1px solid rgba(0,215,163,.5);background:linear-gradient(145deg,rgba(0,215,163,.16),rgba(0,168,255,.06));position:relative;display:grid;place-items:center;clip-path:polygon(15% 0,85% 0,100% 50%,85% 100%,15% 100%,0 50%)}
.brand-symbol:before{content:'';width:15px;height:15px;border:3px solid var(--accent);border-radius:50%;box-shadow:0 0 18px rgba(0,215,163,.45)}
.brand h1{font-size:16px;margin:0;font-weight:800;letter-spacing:.02em}.brand p{font-family:Consolas,monospace;margin:3px 0 0;color:var(--muted);font-size:9px;letter-spacing:.16em;text-transform:uppercase}
.topnav{justify-self:center;display:flex;align-items:center;background:var(--bg);border:1px solid var(--line);padding:4px;border-radius:12px;gap:4px}
.nav-btn{border:0;background:transparent;color:var(--muted);padding:10px 22px;border-radius:8px;font-size:12px;font-weight:800;letter-spacing:.06em;text-transform:uppercase;cursor:pointer;transition:.18s}
.nav-btn:hover{color:var(--text);background:var(--panel2)}.nav-btn.active{background:var(--panel3);color:var(--accent);box-shadow:inset 0 -2px 0 var(--accent)}
.top-actions{display:flex;align-items:center;gap:9px}.theme-btn{border:1px solid var(--line);background:var(--panel2);color:var(--muted);height:42px;padding:0 12px;border-radius:9px;cursor:pointer;font-size:11px;font-weight:700}
.stop-btn{height:48px;border:1px solid #c3283c;background:linear-gradient(#ff5164,#e62f47);color:#fff;border-radius:10px;padding:0 18px;font-weight:900;letter-spacing:.08em;cursor:pointer;box-shadow:0 0 0 3px rgba(255,64,86,.08),0 8px 18px rgba(255,64,86,.18)}

/* shell */
.shell{flex:1;min-height:0;display:flex;flex-direction:column}.viewport{flex:1;min-height:0;overflow:auto;padding:18px}.page-pane{min-height:100%}
.control-layout{display:grid;grid-template-columns:250px minmax(480px,1fr) 295px;gap:14px;min-height:100%}.panel{background:var(--panel);border:1px solid var(--line);border-radius:13px;box-shadow:0 8px 24px var(--shadow);overflow:hidden}.panel-head{padding:12px 14px;border-bottom:1px solid var(--line);display:flex;align-items:center;justify-content:space-between}.panel-kicker{font:700 9px Consolas,monospace;letter-spacing:.16em;text-transform:uppercase;color:var(--muted)}

/* left status tower */
.status-tower{display:flex;flex-direction:column}.machine-state{padding:15px;border-bottom:1px solid var(--line)}.machine-state .big-state{font:800 26px Consolas,monospace;line-height:1;color:var(--accent);margin-top:8px}.badge{display:inline-flex;align-items:center;gap:7px;border:1px solid var(--line);border-radius:999px;padding:6px 9px;font:700 10px Consolas,monospace;margin-top:9px}.badge:before{content:'';width:7px;height:7px;border-radius:50%;background:currentColor}.b-ok{color:var(--accent);border-color:rgba(0,215,163,.32);background:rgba(0,215,163,.06)}.b-err{color:var(--danger);border-color:rgba(255,64,86,.35);background:rgba(255,64,86,.06)}.b-warn{color:var(--warn)}.b-muted{color:var(--muted)}
.position-zone{padding:16px 15px}.position-label{font:700 9px Consolas,monospace;color:var(--muted);letter-spacing:.13em;text-transform:uppercase}.position-number{font:800 42px Consolas,monospace;letter-spacing:-.05em;margin:5px 0 1px}.position-number small{font-size:12px;color:var(--muted);letter-spacing:0}.travel-track{height:164px;display:grid;grid-template-columns:22px 1fr;gap:11px;margin-top:15px}.ruler{font:9px Consolas,monospace;color:var(--muted);display:flex;flex-direction:column;justify-content:space-between;text-align:right}.track{position:relative;border-left:1px solid var(--line);background:linear-gradient(to top,rgba(0,215,163,.12),transparent);border-radius:4px}.track:before,.track:after{content:'';position:absolute;left:-5px;width:9px;height:1px;background:var(--line)}.track:before{top:0}.track:after{bottom:0}.z-fill{position:absolute;left:-2px;bottom:0;width:4px;height:0%;background:linear-gradient(to top,var(--accent),var(--accent2));border-radius:99px;transition:height .25s}.z-fill:after{content:'';position:absolute;top:-4px;left:-4px;width:12px;height:12px;border-radius:50%;background:var(--accent);box-shadow:0 0 14px rgba(0,215,163,.7)}
.mini-stats{padding:10px 15px 15px;margin-top:auto}.mini-stat{display:flex;justify-content:space-between;padding:8px 0;border-top:1px solid var(--line);font:700 10px Consolas,monospace}.mini-stat span:first-child{color:var(--muted);font-weight:600}.mini-stat .v{color:var(--text)}

/* central command deck */
.command-deck{background:transparent;border:none;box-shadow:none;display:flex;flex-direction:column;gap:12px}.mode-strip{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;background:var(--panel);border:1px solid var(--line);border-radius:13px;padding:8px;box-shadow:0 8px 24px var(--shadow)}.subtab-btn{border:1px solid transparent;background:transparent;color:var(--muted);padding:13px 10px;border-radius:9px;font:800 11px Consolas,monospace;letter-spacing:.08em;text-transform:uppercase;cursor:pointer}.subtab-btn:hover{background:var(--panel2);color:var(--text)}.subtab-btn.active{background:rgba(0,215,163,.1);border-color:rgba(0,215,163,.28);color:var(--accent);box-shadow:inset 0 -2px 0 var(--accent)}
.part-console{width:50%;min-width:360px;background:var(--panel);border:1px solid var(--line);border-radius:13px;padding:12px 13px;display:grid;grid-template-columns:105px minmax(0,1fr);gap:10px;align-items:center;box-shadow:0 8px 24px var(--shadow)}.part-console .lab{font:700 9px Consolas,monospace;letter-spacing:.12em;text-transform:uppercase;color:var(--muted)}.part-console select{margin:0}.part-mini{grid-column:2;color:var(--muted);font:10px Consolas,monospace;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.subtab-pane{display:none}.subtab-pane.active{display:flex;flex:1;min-height:0}.operation{width:100%;background:var(--panel);border:1px solid var(--line);border-radius:13px;box-shadow:0 8px 24px var(--shadow);display:flex;flex-direction:column;padding:18px;min-height:390px}.op-top{display:flex;align-items:flex-start;justify-content:space-between;gap:14px}.op-title{font-size:24px;font-weight:800;margin:0}.op-copy{color:var(--muted);font-size:11px;line-height:1.55;margin-top:5px;max-width:560px}.mode-chip{font:700 9px Consolas,monospace;text-transform:uppercase;letter-spacing:.12em;padding:6px 8px;border:1px solid var(--line);color:var(--accent);border-radius:6px;background:var(--panel2)}
.hero-action{flex:1;display:grid;place-items:center;padding:22px 0}.laser-button{width:230px;height:230px;border-radius:50%;border:1px solid rgba(0,215,163,.4);background:radial-gradient(circle at 50% 38%,rgba(0,215,163,.19),rgba(0,215,163,.06) 42%,var(--panel2) 43%);color:var(--text);cursor:pointer;position:relative;box-shadow:0 0 0 10px rgba(0,215,163,.035),0 0 0 20px rgba(0,215,163,.018),inset 0 0 40px rgba(0,215,163,.06);transition:.18s}.laser-button:before{content:'';position:absolute;inset:23px;border:1px dashed rgba(0,215,163,.3);border-radius:50%}.laser-button:hover{transform:scale(1.015);box-shadow:0 0 0 10px rgba(0,215,163,.055),0 0 34px rgba(0,215,163,.18),inset 0 0 50px rgba(0,215,163,.1)}.laser-button strong{display:block;font-size:28px;letter-spacing:.08em}.laser-button span{display:block;color:var(--accent);font:700 10px Consolas,monospace;letter-spacing:.14em;margin-top:8px}
.toggle-box,.sequence-box{background:var(--panel2);border:1px solid var(--line);border-radius:10px;padding:14px}.toggle-row{display:flex;align-items:center;justify-content:space-between}.toggle-label{font-size:12px;font-weight:700}.toggle-copy{font-size:10px;color:var(--muted);margin-top:4px}.toggle{position:relative;width:46px;height:24px}.toggle input{opacity:0;width:0;height:0}.slider{position:absolute;inset:0;border-radius:99px;background:var(--line);transition:.2s}.slider:before{content:'';position:absolute;width:18px;height:18px;left:3px;top:3px;background:white;border-radius:50%;transition:.2s}.toggle input:checked+.slider{background:var(--accent)}.toggle input:checked+.slider:before{transform:translateX(22px)}
.sequence-head{display:flex;justify-content:space-between;font:700 9px Consolas,monospace;color:var(--muted);letter-spacing:.08em;text-transform:uppercase}.sequence-name{font-size:12px;font-weight:800;margin-top:8px}.sequence-copy{font-size:10px;color:var(--muted);margin-top:4px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}.progress-track{height:5px;background:var(--line);border-radius:99px;margin-top:10px;overflow:hidden}.progress-track span{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--accent),var(--accent2));transition:.25s}


/* manual laser program */
.program-bar{width:50%;min-width:360px;display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px}
.manual-workspace{flex:1;display:grid;grid-template-columns:minmax(175px,.8fr) minmax(250px,1.15fr) minmax(220px,.95fr);gap:14px;align-items:stretch;margin-top:14px;min-height:300px}
.manual-left{display:flex;flex-direction:column;justify-content:center;min-width:0}
.manual-center{display:flex;flex-direction:column;align-items:center;justify-content:center;min-width:0}
.manual-right{display:flex;flex-direction:column;justify-content:center;gap:10px;min-width:0}
.program-status{display:grid;grid-template-columns:1fr;gap:8px;margin:0}
.program-stat{background:var(--panel2);border:1px solid var(--line);border-radius:9px;padding:12px}
.program-stat span{display:block;font:700 8px Consolas,monospace;color:var(--muted);letter-spacing:.1em;text-transform:uppercase}
.program-stat strong{display:block;font:800 15px Consolas,monospace;margin-top:5px;color:var(--text)}
.program-message{margin-top:8px;padding:11px 12px;border:1px solid var(--line);border-radius:9px;background:var(--panel2);font:700 10px/1.45 Consolas,monospace;color:var(--muted)}
.program-message.good{color:var(--accent);border-color:rgba(0,215,163,.35)}
.program-message.warn{color:var(--warn);border-color:rgba(255,190,61,.35)}
.program-message.err{color:var(--danger);border-color:rgba(255,64,86,.35)}
.laser-button:disabled{opacity:.26;cursor:not-allowed;filter:grayscale(.75);transform:none!important;box-shadow:none}
.laser-button.armed{border-color:var(--accent);box-shadow:0 0 0 10px rgba(0,215,163,.055),0 0 32px rgba(0,215,163,.2),inset 0 0 50px rgba(0,215,163,.1)}
.pedal-hint{text-align:center;color:var(--muted);font:700 9px Consolas,monospace;letter-spacing:.08em;text-transform:uppercase;margin-top:-10px;padding-bottom:8px}
.config-error{color:var(--danger)!important}
.side-table{width:100%;border-collapse:separate;border-spacing:0 7px;margin-top:10px}.side-table th{text-align:left;font:700 8px Consolas,monospace;color:var(--muted);letter-spacing:.1em;text-transform:uppercase;padding:0 8px}.side-table td{background:var(--panel2);border-top:1px solid var(--line);border-bottom:1px solid var(--line);padding:8px}.side-table td:first-child{border-left:1px solid var(--line);border-radius:8px 0 0 8px;width:70px;font:800 11px Consolas,monospace}.side-table td:last-child{border-right:1px solid var(--line);border-radius:0 8px 8px 0}.side-table input{margin:0}.valid-chip,.invalid-chip{display:inline-block;margin-left:6px;padding:2px 6px;border-radius:5px;font:700 8px Consolas,monospace;text-transform:uppercase}.valid-chip{color:var(--accent);background:rgba(0,215,163,.08);border:1px solid rgba(0,215,163,.25)}.invalid-chip{color:var(--danger);background:rgba(255,64,86,.07);border:1px solid rgba(255,64,86,.25)}

/* auto and jog */
.auto-body{flex:1;display:grid;grid-template-columns:minmax(260px,.85fr) 1.15fr;gap:14px;margin-top:18px}.field-panel,.process-panel{background:var(--panel2);border:1px solid var(--line);border-radius:11px;padding:14px}.field-title{font:700 9px Consolas,monospace;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;margin-bottom:10px}.info{font-size:10px;line-height:1.5;color:var(--accent2);border-left:2px solid var(--accent2);padding-left:9px;margin-bottom:13px}.btn{border:1px solid var(--line);background:var(--panel3);color:var(--text);border-radius:9px;padding:11px 12px;font-size:11px;font-weight:800;cursor:pointer;transition:.15s}.btn:hover:not(:disabled){filter:brightness(1.08)}.btn:disabled{opacity:.35;cursor:not-allowed}.btn-green{background:var(--accent);color:#00291f;border-color:#00b98d}.btn-blue{background:var(--accent2);color:white;border-color:#008bd3}.btn-amber{background:var(--warn);color:#332100;border-color:#dba32d}.btn-red{background:var(--danger);color:white;border-color:#d72c42}.btn-ghost{background:transparent}.btn2{display:grid;grid-template-columns:1fr 1fr;gap:8px}.full{width:100%}.mt8{margin-top:8px}.process-steps{display:grid;gap:8px}.process-step{display:flex;align-items:center;gap:10px;padding:9px;border:1px solid var(--line);border-radius:8px;font:700 10px Consolas,monospace;color:var(--muted)}.process-step i{width:9px;height:9px;border:1px solid var(--line);border-radius:50%;background:var(--panel3)}
.jog-body{flex:1;display:grid;place-items:center}.jog-console{display:grid;grid-template-columns:170px 230px 170px;gap:14px;align-items:center}.jog-arrow{height:170px;border:1px solid var(--line);background:var(--panel2);color:var(--accent2);font-size:54px;border-radius:16px;cursor:pointer;transition:.16s}.jog-arrow:hover{border-color:var(--accent2);box-shadow:0 0 28px rgba(0,168,255,.12);transform:translateY(-2px)}.jog-centre{text-align:center}.jog-centre label{display:block;font:700 9px Consolas,monospace;color:var(--muted);letter-spacing:.12em;text-transform:uppercase;margin-bottom:8px}.jog-centre input{text-align:center;font:800 34px Consolas,monospace;padding:14px}.jog-stop{margin-top:10px;width:100%}

/* io rack */
.io-rack{display:flex;flex-direction:column}.io-summary{padding:14px;border-bottom:1px solid var(--line)}.io-summary h3{font-size:13px;margin:0}.io-summary p{font:9px Consolas,monospace;color:var(--muted);margin:4px 0 0}.io-group{padding:12px 14px;border-bottom:1px solid var(--line)}.io-group-title{font:700 9px Consolas,monospace;letter-spacing:.12em;text-transform:uppercase;color:var(--muted);margin-bottom:7px}.sig-row{display:flex;align-items:center;justify-content:space-between;padding:9px 0;border-top:1px solid rgba(127,150,145,.1)}.sig-row:first-of-type{border-top:0}.sig-name{font-size:11px;font-weight:700}.sig-sub{font:9px Consolas,monospace;color:var(--muted);margin-top:2px}.dot{width:11px;height:11px;border-radius:50%;border:1px solid var(--line);background:var(--panel3)}.dot.hi{background:var(--accent);border-color:var(--accent);box-shadow:0 0 12px rgba(0,215,163,.6)}.dot.lo{background:var(--panel3)}.motor-grid{padding:12px 14px;display:grid;grid-template-columns:1fr 1fr;gap:8px}.motor-stat{background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:9px}.motor-stat.wide{grid-column:1/-1}.motor-label{font:700 8px Consolas,monospace;color:var(--muted);letter-spacing:.1em;text-transform:uppercase}.motor-value{font:800 20px Consolas,monospace;margin-top:4px}.motor-unit{font-size:9px;color:var(--muted);margin-left:3px}

/* settings */
.settings-wrap{max-width:1180px;margin:0 auto}.settings-hero{margin-bottom:14px;display:flex;justify-content:space-between;align-items:end}.settings-hero h2{font-size:28px;margin:0}.settings-hero p{color:var(--muted);font-size:11px;margin:6px 0 0}.settings-grid{display:grid;grid-template-columns:1fr 1.15fr;gap:14px}.settings-card{background:var(--panel);border:1px solid var(--line);border-radius:13px;padding:15px;box-shadow:0 8px 24px var(--shadow);margin-bottom:14px}.settings-title{font:700 10px Consolas,monospace;color:var(--muted);letter-spacing:.14em;text-transform:uppercase;margin-bottom:13px}.homing-status{display:flex;align-items:center;gap:8px;background:var(--panel2);border:1px solid var(--line);padding:10px;border-radius:8px;font-size:11px;margin-bottom:12px}.form-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}.form-grid .span2{grid-column:1/-1}label{display:block;font-size:10px;color:var(--muted);font-weight:700;margin-bottom:5px}input[type=text],input[type=number],select{width:100%;background:var(--input);border:1px solid var(--line);border-radius:8px;color:var(--text);padding:9px 10px;outline:none;font-size:12px}input:focus,select:focus{border-color:var(--accent);box-shadow:0 0 0 3px rgba(0,215,163,.08)}input:disabled{opacity:.55}.help{font-size:9px;color:var(--muted);line-height:1.5;margin:7px 0 10px}.part-item{display:flex;justify-content:space-between;align-items:center;background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:10px;margin-bottom:7px;cursor:pointer}.part-item:hover,.part-item.sel{border-color:rgba(0,215,163,.45)}.part-item.sel{box-shadow:inset 3px 0 0 var(--accent)}.part-name{font-size:11px;font-weight:800}.part-sub{font:9px Consolas,monospace;color:var(--muted);margin-top:3px}.part-acts{display:flex;gap:5px}.icon-btn{border:1px solid var(--line);background:var(--panel);color:var(--muted);border-radius:6px;padding:5px 8px;cursor:pointer}.icon-btn:hover{color:var(--accent);border-color:var(--accent)}.icon-btn.del:hover{color:var(--danger);border-color:var(--danger)}.step-row{display:grid;grid-template-columns:1fr 1fr auto;gap:8px;align-items:end;background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:9px;margin-bottom:8px}.divider{border:0;border-top:1px solid var(--line);margin:12px 0}

/* terminal */
.log-wrap{height:var(--terminal);flex:none;border-top:1px solid var(--line);background:#05090b;display:flex;flex-direction:column}.log-hdr{height:32px;flex:none;display:flex;align-items:center;justify-content:space-between;padding:0 15px;border-bottom:1px solid #152327}.log-title{font:700 9px Consolas,monospace;color:#66827c;letter-spacing:.15em;text-transform:uppercase}.log-clear{border:1px solid #21363a;background:#0a1215;color:#6f8d87;border-radius:5px;padding:3px 8px;font:700 9px Consolas,monospace;cursor:pointer}.log-box{flex:1;overflow-y:auto;padding:8px 15px;font:10px/1.45 Consolas,monospace;color:#6f8d87}.ll-ok{color:#00d7a3}.ll-err{color:#ff4056}.ll-warn{color:#ffbe3d}

/* modal */
.overlay{display:none;position:fixed;inset:0;background:rgba(0,0,0,.72);backdrop-filter:blur(4px);z-index:200;align-items:center;justify-content:center}.overlay.open{display:flex}.modal{width:520px;max-height:82vh;overflow:auto;background:var(--panel);border:1px solid var(--line);border-radius:14px;padding:18px;box-shadow:0 24px 70px rgba(0,0,0,.5)}.modal h2{margin:0 0 14px;font-size:18px}.modal-foot{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-top:12px}
::-webkit-scrollbar{width:6px;height:6px}::-webkit-scrollbar-thumb{background:var(--line);border-radius:8px}
@media(max-width:1180px){.control-layout{grid-template-columns:220px 1fr}.io-rack{grid-column:1/-1;display:grid;grid-template-columns:1fr 1fr 1fr}.io-summary{display:none}.motor-grid{padding:12px}.jog-console{grid-template-columns:140px 200px 140px}.manual-workspace{grid-template-columns:minmax(170px,.75fr) minmax(240px,1fr) minmax(210px,.9fr)}}
.hmi-toast{position:fixed;right:18px;top:88px;z-index:500;max-width:440px;padding:12px 16px;border-radius:10px;border:1px solid var(--line);background:var(--panel);box-shadow:0 14px 40px rgba(0,0,0,.4);font:700 12px/1.45 Consolas,monospace;opacity:0;transform:translateY(-8px);pointer-events:none;transition:.18s}.hmi-toast.show{opacity:1;transform:translateY(0)}.hmi-toast.err{border-color:rgba(255,64,86,.55);color:var(--danger)}.hmi-toast.warn{border-color:rgba(255,190,61,.55);color:var(--warn)}.hmi-toast.good{border-color:rgba(45,220,130,.45);color:var(--accent)}
.confirm-copy{font-size:13px;line-height:1.6;color:var(--text);margin:8px 0 12px}.confirm-warning{padding:10px 12px;border:1px solid rgba(255,190,61,.35);background:rgba(255,190,61,.08);border-radius:8px;color:var(--warn);font-size:12px;line-height:1.5}
@media(max-width:900px){body{overflow:auto}.topbar{grid-template-columns:1fr auto;height:auto;min-height:76px}.topnav{grid-column:1/-1;grid-row:2;margin:8px auto}.shell{min-height:800px}.control-layout{grid-template-columns:1fr}.status-tower{display:grid;grid-template-columns:1fr 1fr}.position-zone{border-left:1px solid var(--line)}.io-rack{display:block}.settings-grid{grid-template-columns:1fr}.auto-body{grid-template-columns:1fr}.jog-console{grid-template-columns:1fr}.jog-arrow{height:90px}.part-console,.program-bar{width:100%;min-width:0}.manual-workspace{grid-template-columns:1fr}.manual-left,.manual-right{justify-content:flex-start}.manual-right{display:grid;grid-template-columns:1fr 1fr}.manual-center{padding:8px 0}}
@media(max-width:620px){.manual-right{grid-template-columns:1fr}}
</style>
</head>
<body>
<header class="topbar">
  <div class="brand"><div class="brand-symbol"></div><div><h1>Laser Cell Console</h1><p>Z-axis marking station / operator terminal</p></div></div>
  <nav class="topnav">
    <button class="nav-btn active" id="nav-control" onclick="switchPage('control',this)">Operate</button>
    <button class="nav-btn" id="nav-settings" onclick="switchPage('settings',this)">Configure</button>
  </nav>
  <div class="top-actions">
    <button class="theme-btn" id="theme-btn" onclick="toggleTheme()">Theme</button>
    <button class="stop-btn" onclick="doStop()">EMERGENCY STOP</button>
  </div>
</header>

<div class="shell">
  <main class="viewport">
    <section id="page-control" class="page-pane">
      <div class="control-layout">
        <aside class="panel status-tower">
          <div class="panel-head"><span class="panel-kicker">Machine status</span></div>
          <div class="machine-state">
            <div class="panel-kicker">Controller state</div>
            <div class="big-state" id="ov-state">IDLE</div>
            <span id="hb-conn" class="badge b-err">Drive: disconnected</span>
            <span id="hb-state" class="badge b-muted">IDLE</span>
          </div>
          <div class="position-zone">
            <div class="position-label">Absolute Z position</div>
            <div class="position-number"><span id="ov-pos">--</span><small> mm</small></div>
            <div class="travel-track">
              <div class="ruler"><span>{{ max_travel }}</span><span>{{ max_travel/2 }}</span><span>0</span></div>
              <div class="track"><div class="z-fill" id="z-fill"></div></div>
            </div>
          </div>
          <div class="mini-stats">
            <div class="mini-stat"><span>PART</span><span class="v" id="ov-part">none</span></div>
            <div class="mini-stat"><span>STEP</span><span class="v" id="ov-progress">0 / 0</span></div>
          </div>
        </aside>

        <section class="command-deck">
          <div class="mode-strip">
            <button class="subtab-btn active" id="st-manual" onclick="switchSub('manual',this)">Manual mark</button>
            <button class="subtab-btn" id="st-auto" onclick="switchSub('auto',this)">Auto cycle</button>
            <button class="subtab-btn" id="st-jog" onclick="switchSub('jog',this)">Axis jog</button>
          </div>

          <div class="part-console">
            <div class="lab">Loaded recipe</div>
            <select id="part-select" onchange="onPartSelect(this.value)"><option value="">-- select part --</option></select>
            <div></div><div id="part-mini" class="part-mini">No marking recipe selected</div>
          </div>

          <div id="sub-manual" class="subtab-pane active">
            <div class="operation">
              <div class="op-top">
                <div>
                  <h2 class="op-title">Manual Laser Program</h2>
                  <div class="op-copy">Select a valid part, start the laser program to position Z at Side 1, then start each part with the screen MARK button or GPIO22 foot pedal. OUT4 advances intermediate sides; OUT5 completes the part.</div>
                </div>
                <span class="mode-chip" id="manual-mode-chip">not started</span>
              </div>

              <div class="program-bar">
                <button class="btn btn-green" id="btn-program-start" onclick="manualProgramStart()">LASER PROGRAM START</button>
                <button class="btn btn-red" id="btn-program-stop" onclick="manualProgramStop()">STOP</button>
                <button class="btn btn-blue" id="btn-program-resume" onclick="manualProgramResume()">RESUME</button>
                <button class="btn btn-amber" id="btn-program-reset" onclick="manualProgramReset()">RESET</button>
              </div>

              <div class="manual-workspace">
                <div class="manual-left">
                  <div class="program-status">
                    <div class="program-stat"><span>Program</span><strong id="manual-program-state">NO PART</strong></div>
                    <div class="program-stat"><span>Side</span><strong id="manual-side-status">0 / 0</strong></div>
                    <div class="program-stat"><span>OUT4 count</span><strong id="manual-out4-status">0</strong></div>
                  </div>
                  <div class="program-message" id="manual-program-message">Select a configured part to begin.</div>
                </div>

                <div class="manual-center">
                  <div class="hero-action">
                    <button class="laser-button" id="btn-manual-laser" onclick="manualMark()" disabled><strong>MARK</strong><span>START CURRENT PART</span></button>
                  </div>
                  <div class="pedal-hint" id="pedal-hint">GPIO22 FOOT PEDAL LOCKED</div>
                </div>

                <div class="manual-right">
                  <div class="toggle-box">
                    <div class="sequence-head"><span>Operator start</span><span id="pedal-state">LOCKED</span></div>
                    <div class="sequence-name">Screen MARK or foot pedal</div>
                    <div class="sequence-copy">The pedal is accepted only while Z is at the Side 1 initial position. A pedal press does not re-pulse GPIO17.</div>
                  </div>
                  <div class="sequence-box">
                    <div class="sequence-head"><span>Current recipe</span><span id="manual-step">0 / 0</span></div>
                    <div class="sequence-name" id="manual-summary-title">Select a part</div>
                    <div class="sequence-copy" id="manual-summary-copy">Every side requires a configured Z height.</div>
                    <div class="progress-track"><span id="manual-progress"></span></div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div id="sub-auto" class="subtab-pane">
            <div class="operation">
              <div class="op-top"><div><h2 class="op-title">Automatic Cycle</h2><div class="op-copy">Run repeated laser cycles with robot placement and pickup handshake placeholders.</div></div><span class="mode-chip">sequence mode</span></div>
              <div class="auto-body">
                <div class="field-panel">
                  <div class="field-title">Production quantity</div>
                  <label>Number of parts to laser</label>
                  <input type="number" id="cycle-count" min="1" max="9999" step="1" value="1">
                  <div class="info">Robot OPC UA communication is still configured as a placeholder in the controller.</div>
                  <button class="btn btn-green full" id="btn-auto-start" onclick="startAuto()">START AUTOMATIC CYCLE</button>
                  <div class="btn2 mt8"><button class="btn btn-amber" id="btn-pause" onclick="pauseResume()">PAUSE</button><button class="btn btn-red" onclick="doStop()">STOP</button></div>
                </div>
                <div class="process-panel">
                  <div class="field-title">Sequence path</div>
                  <div class="process-steps">
                    <div class="process-step"><i></i><span>01 / Robot places part</span></div>
                    <div class="process-step"><i></i><span>02 / Z axis moves to recipe position</span></div>
                    <div class="process-step"><i></i><span>03 / Laser marking cycle</span></div>
                    <div class="process-step"><i></i><span>04 / Remaining recipe positions</span></div>
                    <div class="process-step"><i></i><span>05 / Robot picks finished part</span></div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div id="sub-jog" class="subtab-pane">
            <div class="operation">
              <div class="op-top"><div><h2 class="op-title">Z Axis Jog</h2><div class="op-copy">Move the vertical axis by a defined relative distance. Travel is limited by the controller to the configured machine range.</div></div><span class="mode-chip">service motion</span></div>
              <div class="jog-body">
                <div class="jog-console">
                  <button class="jog-arrow" id="btn-jog-down" onclick="doJog('down')">&#9660;</button>
                  <div class="jog-centre"><label>Jog distance / mm</label><input type="number" id="jog-dist" min="0.1" max="400" step="0.1" value="1"><button class="btn btn-red jog-stop" onclick="doStop()">STOP AXIS</button></div>
                  <button class="jog-arrow" id="btn-jog-up" onclick="doJog('up')">&#9650;</button>
                </div>
              </div>
            </div>
          </div>
        </section>

        <aside class="panel io-rack">
          <div class="io-summary"><h3>Live I/O Rack</h3><p>GPIO and motor feedback / guarded 1 s refresh</p></div>
          <div class="io-group"><div class="io-group-title">Laser / operator inputs</div>
            <div class="sig-row"><div><div class="sig-name">Intermediate finish</div><div class="sig-sub">GPIO 26 &lt;- EzCad2 OUT4</div></div><span class="dot" id="sig-26"></span></div>
            <div class="sig-row"><div><div class="sig-name">Part complete</div><div class="sig-sub">GPIO 4 &lt;- EzCad2 OUT5</div></div><span class="dot" id="sig-4"></span></div>
            <div class="sig-row"><div><div class="sig-name">Foot pedal</div><div class="sig-sub">GPIO 22 &lt;- pedal acknowledgement</div></div><span class="dot" id="sig-22"></span></div>
          </div>
          <div class="io-group"><div class="io-group-title">Laser outputs</div>
            <div class="sig-row"><div><div class="sig-name">Laser start</div><div class="sig-sub">GPIO 17 -&gt; GIN15</div></div><span class="dot" id="sig-17"></span></div>
            <div class="sig-row"><div><div class="sig-name">Remarking</div><div class="sig-sub">GPIO 27 -&gt; pin 8</div></div><span class="dot" id="sig-27"></span></div>
          </div>
          <div class="motor-grid">
            <div class="motor-stat"><div class="motor-label">Position</div><div class="motor-value"><span id="m-pos">--</span><span class="motor-unit">mm</span></div></div>
            <div class="motor-stat"><div class="motor-label">Speed</div><div class="motor-value"><span id="m-spd">--</span><span class="motor-unit">rpm</span></div></div>
            <div class="motor-stat wide"><div class="motor-label">Motion state</div><div class="motor-value" id="m-state" style="font-size:13px">--</div></div>
          </div>
        </aside>
      </div>
    </section>

    <section id="page-settings" class="page-pane" style="display:none">
      <div class="settings-wrap">
        <div class="settings-hero"><div><h2>Machine Configuration</h2><p>Service parameters, homing behavior, laser timing and marking recipes.</p></div><span class="mode-chip">engineering access</span></div>
        <div class="settings-grid">
          <div>
            <div class="settings-card">
              <div class="settings-title">Z axis homing</div>
              <div class="homing-status" id="homing-status"><span class="dot" id="home-dot"></span><span id="home-status-text">Unknown - home to verify</span></div>
              <div class="form-grid">
                <div><label>Fast search speed / rpm</label><input type="number" id="h-fast" min="5" max="200" step="5"></div>
                <div><label>Slow approach / rpm</label><input type="number" id="h-slow" min="1" max="50" step="1"></div>
                <div class="span2"><label>Homing timeout / seconds</label><input type="number" id="h-timeout" min="10" max="600" step="10"></div>
              </div>
              <div class="help">Offset is fixed at -2.5 mm. The axis moves to 0 mm after a successful homing sequence.</div>
              <div class="btn2"><button class="btn btn-amber" id="btn-home" onclick="startHoming()">START HOMING</button><button class="btn btn-red" id="btn-hstop" onclick="stopHoming()">STOP HOMING</button></div>
              <button class="btn btn-ghost full mt8" onclick="saveHomingSettings()">SAVE HOMING PARAMETERS</button>
            </div>
            <div class="settings-card">
              <div class="settings-title">Laser trigger</div>
              <label>Laser trigger pulse / ms</label><input type="number" id="pulse-ms" min="50" max="2000" step="50" value="100">
              <div class="help">Duration GPIO 17 is held HIGH to trigger the marking start input.</div>
              <button class="btn btn-ghost full" onclick="saveLaserSettings()">SAVE LASER PARAMETER</button>
            </div>
          </div>
          <div class="settings-card" style="margin-bottom:0">
            <div class="settings-title">Part recipes</div>
            <div id="part-list-settings"></div>
            <button class="btn btn-ghost full mt8" onclick="openEditor(null)">CREATE NEW PART</button>
            <div id="part-detail-settings" style="margin-top:12px"><div class="help">Select a part to inspect its marking positions.</div></div>
          </div>
        </div>
      </div>
    </section>
  </main>

  <div class="log-wrap"><div class="log-hdr"><span class="log-title">Machine event terminal</span><button class="log-clear" onclick="clearLog()">CLEAR BUFFER</button></div><div class="log-box" id="log-box"></div></div>
</div>

<div class="overlay" id="modal"><div class="modal">
  <h2 id="modal-title">New part</h2>
  <label>Part name</label><input type="text" id="edit-name" placeholder="e.g. Part_A">
  <div style="margin-top:12px"><label>Number of sides to be lasered</label><input type="number" id="edit-sides" min="1" max="64" step="1" value="1" onchange="renderSideRows()" oninput="renderSideRows()"></div>
  <div class="help">Required: every side must have a Z height. Different sides may use the same height.</div>
  <table class="side-table"><thead><tr><th>Side</th><th>Required Z height / mm</th></tr></thead><tbody id="steps-wrap"></tbody></table>
  <div class="modal-foot"><button class="btn btn-green" onclick="savePart()">SAVE PART</button><button class="btn btn-ghost" onclick="closeModal()">CANCEL</button></div>
</div></div>

<div class="overlay" id="action-modal">
  <div class="modal" style="width:460px">
    <h2 id="action-modal-title">Confirm action</h2>
    <div class="confirm-copy" id="action-modal-message"></div>
    <div class="confirm-warning" id="action-modal-warning" style="display:none"></div>
    <div class="modal-foot">
      <button class="btn btn-red" id="action-modal-confirm" onclick="runActionConfirm()">CONFIRM</button>
      <button class="btn btn-ghost" onclick="closeActionConfirm()">CANCEL</button>
    </div>
  </div>
</div>
<div class="hmi-toast" id="hmi-toast"></div>

<script>
const MAX_TRAVEL = {{ max_travel }};
const PART_HEIGHT_MIN = {{ part_height_min }};
const PART_HEIGHT_MAX = {{ part_height_max }};
let lastStatus=null;
let selPart = null;
let pollRunning=false;
let toastTimer=null;
let pendingConfirmAction=null;

function showToast(message,kind='err',ms=4500){
  const el=document.getElementById('hmi-toast');
  if(!el) return;
  el.textContent=message;
  el.className='hmi-toast show '+kind;
  if(toastTimer) clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>{el.className='hmi-toast';},ms);
}
function openActionConfirm(title,message,onConfirm,opts={}){
  pendingConfirmAction=onConfirm;
  document.getElementById('action-modal-title').textContent=title;
  document.getElementById('action-modal-message').textContent=message;
  const w=document.getElementById('action-modal-warning');
  if(opts.warning){w.textContent=opts.warning;w.style.display='block';}
  else{w.textContent='';w.style.display='none';}
  document.getElementById('action-modal-confirm').textContent=opts.confirmText||'CONFIRM';
  document.getElementById('action-modal').classList.add('open');
}
function closeActionConfirm(){
  document.getElementById('action-modal').classList.remove('open');
  pendingConfirmAction=null;
}
function runActionConfirm(){
  const fn=pendingConfirmAction;
  document.getElementById('action-modal').classList.remove('open');
  pendingConfirmAction=null;
  if(fn) fn();
}
async function fetchWithTimeout(url,options={},timeoutMs=3000){
  const ctrl=new AbortController();
  const timer=setTimeout(()=>ctrl.abort(),timeoutMs);
  try{return await fetch(url,{...options,signal:ctrl.signal});}
  finally{clearTimeout(timer);}
}

// ── Theme ─────────────────────────────────────────────────────────────────
function applyTheme(t){
  document.documentElement.setAttribute('data-theme', t);
  document.getElementById('theme-btn').textContent = t==='light' ? '🌙 Dark' : '☀️ Light';
}
function toggleTheme(){
  const cur = document.documentElement.getAttribute('data-theme');
  const next = cur==='dark' ? 'light' : 'dark';
  applyTheme(next);
  fetch('/api/settings/theme',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({theme:next})});
}
applyTheme(document.documentElement.getAttribute('data-theme')||'light');

// ── Pages ─────────────────────────────────────────────────────────────────
function switchPage(id, btn){
  document.querySelectorAll('.page-pane').forEach(p=>p.style.display='none');
  document.querySelectorAll('.nav-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById('page-'+id).style.display='block';
  btn.classList.add('active');
}

// ── Sub-tabs ──────────────────────────────────────────────────────────────
function switchSub(id, btn){
  document.querySelectorAll('.subtab-pane').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.subtab-btn').forEach(b=>b.classList.remove('active'));
  document.getElementById('sub-'+id).classList.add('active');
  btn.classList.add('active');
}

// ── Poll ──────────────────────────────────────────────────────────────────
async function poll(){
  if(pollRunning) return;
  pollRunning=true;
  try{
    const [sr,gr,lr]=await Promise.allSettled([
      fetchWithTimeout('/api/status',{},2500),
      fetchWithTimeout('/api/signals',{},2500),
      fetchWithTimeout('/api/logs',{},2500)
    ]);
    if(sr.status==='fulfilled'&&sr.value.ok) updateStatus(await sr.value.json());
    if(gr.status==='fulfilled'&&gr.value.ok) updateSignals(await gr.value.json());
    if(lr.status==='fulfilled'&&lr.value.ok){const d=await lr.value.json();updateLog(d.logs||[]);}
  }catch(_e){}
  finally{pollRunning=false;}
}
setInterval(poll,1000); poll();

// ── Status ────────────────────────────────────────────────────────────────
const SC={
  IDLE:'var(--muted)',HOMING:'var(--warn)',READY:'var(--accent)',
  PROGRAM_SELECTED:'var(--accent2)',INITIALIZING:'var(--accent2)',
  READY_TO_MARK:'var(--accent)',MOVING_Z:'var(--accent2)',
  LASER_FIRING:'var(--purple)',WAITING_LASER:'var(--purple)',
  WAITING_ROBOT:'var(--accent2)',PROGRAM_STOPPED:'var(--warn)',
  RESETTING:'var(--warn)',PAUSED:'var(--warn)',ERROR:'var(--danger)'
};

function updateStatus(d){
  lastStatus=d;
  const cb=document.getElementById('hb-conn');
  cb.className='badge '+(d.connected?'b-ok':'b-err');
  cb.textContent=d.connected?'Drive: connected':'Drive: disconnected';
  const sb=document.getElementById('hb-state');
  sb.textContent=d.state; sb.style.color=SC[d.state]||'var(--muted)';

  document.getElementById('m-pos').textContent=d.z_position!==null?d.z_position.toFixed(2):'--';
  document.getElementById('m-spd').textContent=d.speed_rpm!==null?d.speed_rpm:'--';
  document.getElementById('m-state').textContent=d.state||'--';
  document.getElementById('m-state').style.color=SC[d.state]||'var(--muted)';

  const ovs=document.getElementById('ov-state'); if(ovs){ovs.textContent=d.state||'--';ovs.style.color=SC[d.state]||'var(--text)';}
  const ovp=document.getElementById('ov-pos'); if(ovp) ovp.textContent=d.z_position!==null?d.z_position.toFixed(2):'--';
  const zf=document.getElementById('z-fill'); if(zf){const pct=d.z_position!==null?Math.max(0,Math.min(100,(d.z_position/MAX_TRAVEL)*100)):0;zf.style.height=pct+'%';}
  const ovpart=document.getElementById('ov-part'); if(ovpart) ovpart.textContent=d.part||document.getElementById('part-select').value||'No part selected';

  const side=d.manual_side_number||0,total=d.manual_total_sides||0;
  const ovprog=document.getElementById('ov-progress'); if(ovprog) ovprog.textContent=`${side} / ${total}`;
  const manstep=document.getElementById('manual-step'); if(manstep) manstep.textContent=`${side} / ${total}`;
  const manbar=document.getElementById('manual-progress'); if(manbar) manbar.style.width=(total?Math.min(100,(Math.max(0,side-1)/total)*100):0)+'%';
  const pstate=document.getElementById('manual-program-state'); if(pstate) pstate.textContent=(d.manual_program_state||'NO_PART').replaceAll('_',' ');
  const sstat=document.getElementById('manual-side-status'); if(sstat) sstat.textContent=`${side} / ${total}`;
  const o4=document.getElementById('manual-out4-status'); if(o4) o4.textContent=d.manual_out4_count||0;
  const chip=document.getElementById('manual-mode-chip'); if(chip) chip.textContent=(d.manual_program_state||'not started').replaceAll('_',' ');

  const msg=document.getElementById('manual-program-message');
  if(msg){
    msg.className='program-message';
    if(d.manual_error){msg.textContent=d.manual_error;msg.classList.add('err');}
    else if(d.manual_stopped && d.manual_awaiting_result){msg.textContent='Program stopped while waiting for EzCad2. RESUME keeps waiting, or RESET can recover after you confirm EzCad2 has stopped.';msg.classList.add('warn');}
    else if(d.manual_stopped){msg.textContent='Program stopped. Z holds position. RESUME continues from the stored current side.';msg.classList.add('warn');}
    else if(d.manual_mark_enabled){msg.textContent='Ready for a new part. Press MARK or the GPIO22 foot pedal.';msg.classList.add('good');}
    else if(d.manual_awaiting_result){msg.textContent=`Side ${side}/${total} is active. Waiting for ${side<total?'OUT4 intermediate finish':'OUT5 part complete'}.`;msg.classList.add('warn');}
    else if(d.manual_program_state==='INITIALIZING'||d.manual_program_state==='RETURNING_INITIAL'||d.manual_program_state==='MOVING_NEXT'||d.manual_program_state==='RESETTING'){msg.textContent='Z axis positioning in progress. Operator start is locked.';msg.classList.add('warn');}
    else if(d.manual_program_started){msg.textContent='Laser Program started.';msg.classList.add('good');}
    else {msg.textContent='Select a valid part, then press LASER PROGRAM START.';}
  }

  const mark=document.getElementById('btn-manual-laser');
  if(mark){mark.disabled=!d.manual_mark_enabled;mark.classList.toggle('armed',!!d.manual_mark_enabled);}
  const pedalHint=document.getElementById('pedal-hint'); if(pedalHint) pedalHint.textContent=d.manual_mark_enabled?'GPIO22 FOOT PEDAL ENABLED':'GPIO22 FOOT PEDAL LOCKED';
  const pedalState=document.getElementById('pedal-state'); if(pedalState) pedalState.textContent=d.manual_mark_enabled?'ENABLED':'LOCKED';

  const startBtn=document.getElementById('btn-program-start');
  const stopBtn=document.getElementById('btn-program-stop');
  const resumeBtn=document.getElementById('btn-program-resume');
  const resetBtn=document.getElementById('btn-program-reset');
  const hasPart=!!document.getElementById('part-select').value;
  if(startBtn) startBtn.disabled=!hasPart||!d.position_ready||d.manual_program_started;
  if(stopBtn) stopBtn.disabled=!d.manual_program_started||d.manual_stopped;
  if(resumeBtn) resumeBtn.disabled=!d.manual_program_started||!d.manual_stopped;
  // During a missing-OUT4/OUT5 recovery, RESET is intentionally enabled
  // only after STOP. This prevents Z motion while EzCad2 may still be active.
  if(resetBtn) resetBtn.disabled=!d.manual_program_started||(d.manual_awaiting_result&&!d.manual_stopped);

  const homeDot=document.getElementById('home-dot'),homeTxt=document.getElementById('home-status-text');
  if(d.state==='HOMING'){homeDot.className='dot hi';homeDot.style.background='var(--warn)';homeTxt.textContent='Homing in progress...';}
  else if(d.position_ready){homeDot.className='dot hi';homeDot.style.background='var(--accent)';homeTxt.textContent=`Absolute position valid · ${d.z_position!==null?d.z_position.toFixed(2)+'mm':'--'}`;}
  else if(d.encoder_position_available){homeDot.className='dot hi';homeDot.style.background='var(--warn)';homeTxt.textContent=`Encoder online, reference required · raw ${d.z_position_raw!==null?d.z_position_raw:'--'}`;}
  else{homeDot.className='dot lo';homeTxt.textContent='Absolute position unavailable - check drive / home machine';}

  const bh=document.getElementById('btn-home'),bhs=document.getElementById('btn-hstop');
  if(bh) bh.disabled=d.manual_program_started||d.state==='HOMING';
  if(bhs) bhs.disabled=d.state!=='HOMING';
  const bas=document.getElementById('btn-auto-start'); if(bas) bas.disabled=!d.position_ready||d.manual_program_started;
  const bju=document.getElementById('btn-jog-up'),bjd=document.getElementById('btn-jog-down');
  if(bju) bju.disabled=!d.position_ready||d.manual_program_started;
  if(bjd) bjd.disabled=!d.position_ready||d.manual_program_started;
}

// ── Signals ───────────────────────────────────────────────────────────────
function updateSignals(d){
  setDot('sig-26',d.gpio26); setDot('sig-4',d.gpio4); setDot('sig-22',d.gpio22);
  setDot('sig-17',d.gpio17); setDot('sig-27',d.gpio27);
}
function setDot(id,hi){
  const el=document.getElementById(id);
  if(el) el.className='dot '+(hi?'hi':'lo');
}

// ── Log ───────────────────────────────────────────────────────────────────
function updateLog(lines){
  const box=document.getElementById('log-box');
  const atBot=box.scrollHeight-box.scrollTop<=box.clientHeight+20;
  box.innerHTML=lines.map(l=>{
    const c=/error|Error|FAULT|STOP/i.test(l)?'ll-err'
            :/COMPLETE|complete|READY|done|homed/i.test(l)?'ll-ok'
            :/timeout|WARN|cancel|placeholder/i.test(l)?'ll-warn':'';
    return `<div class="ll ${c}">${esc(l)}</div>`;
  }).join('');
  if(atBot) box.scrollTop=box.scrollHeight;
}
function clearLog(){ fetch('/api/logs/clear',{method:'POST'}); }

// ── Parts ─────────────────────────────────────────────────────────────────
function loadParts(){
  fetch('/api/parts').then(r=>r.json()).then(d=>{renderPartListSettings(d.parts);populateSelect(d.parts);}).catch(()=>{});
}
setInterval(loadParts,3000); loadParts();

function populateSelect(parts){
  const sel=document.getElementById('part-select'),cur=sel.value;
  sel.innerHTML='<option value="">-- select part --</option>'+parts.map(p=>`<option value="${ea(p.name)}">${esc(p.name)}${p.valid?'':' [CONFIG ERROR]'}</option>`).join('');
  if(cur) sel.value=cur;
}

function onPartSelect(name){
  selPart=name||null;
  if(!name){
    document.getElementById('part-mini').textContent='No marking recipe selected';
    document.getElementById('manual-summary-title').textContent='Select a part';
    document.getElementById('manual-summary-copy').textContent='Every side requires a configured Z height.';
    fetch('/api/manual/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({part:''})});
    return;
  }
  fetch(`/api/parts/${encodeURIComponent(name)}`).then(r=>r.json()).then(p=>{
    const positions=Array.isArray(p.positions)?p.positions:[];
    const sides=Number(p.sides||0);
    const valid=sides>=1&&positions.length===sides&&positions.every(x=>x.mm!==undefined&&x.mm!==null&&x.mm!=='');
    if(!valid){
      const err=`Part configuration incomplete: a side count and one height for every side are mandatory.`;
      document.getElementById('part-mini').textContent=err;
      document.getElementById('part-mini').classList.add('config-error');
      document.getElementById('manual-summary-title').textContent=p.name;
      document.getElementById('manual-summary-copy').textContent=err;
    }else{
      document.getElementById('part-mini').classList.remove('config-error');
      document.getElementById('part-mini').textContent=`${sides} side${sides!==1?'s':''} · `+positions.map((x,i)=>`S${i+1}:${x.mm}mm`).join(' · ');
      document.getElementById('manual-summary-title').textContent=p.name;
      document.getElementById('manual-summary-copy').textContent=positions.map((x,i)=>`Side ${i+1} · ${x.mm} mm`).join('  •  ');
    }
    renderPartDetailSettings(p);
  }).catch(()=>{});
  fetch('/api/manual/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({part:name})})
    .then(async r=>({ok:r.ok,data:await r.json()})).then(x=>{if(!x.ok||!x.data.ok) showToast(x.data.error||'Part selection rejected','err');}).catch(()=>showToast('Part selection request failed','err'));
  loadParts();
}

function renderPartListSettings(parts){
  const el=document.getElementById('part-list-settings');
  if(!parts.length){el.innerHTML='<div class="help">No parts yet</div>';return;}
  el.innerHTML=parts.map(p=>`<div class="part-item ${selPart===p.name?'sel':''}" onclick="selectPartSettings('${ea(p.name)}')"><div><div class="part-name">${esc(p.name)} ${p.valid?'<span class="valid-chip">valid</span>':'<span class="invalid-chip">incomplete</span>'}</div><div class="part-sub">${p.sides||0} side${p.sides!==1?'s':''}${p.valid?'':' · '+esc(p.error||'configuration error')}</div></div><div class="part-acts"><button class="icon-btn" onclick="event.stopPropagation();openEditor('${ea(p.name)}')">EDIT</button><button class="icon-btn del" onclick="event.stopPropagation();deletePart('${ea(p.name)}')">X</button></div></div>`).join('');
}

function selectPartSettings(name){
  selPart=name;document.getElementById('part-select').value=name;onPartSelect(name);loadParts();
}

function renderPartDetailSettings(p){
  const positions=Array.isArray(p.positions)?p.positions:[];
  document.getElementById('part-detail-settings').innerHTML=`<div class="divider"></div><div style="font-weight:800;font-size:13px;margin-bottom:8px">${esc(p.name)} · ${p.sides||0} sides</div>${positions.map((pos,i)=>`<div style="background:var(--panel2);border:1px solid var(--line);border-radius:8px;padding:8px 10px;margin-bottom:5px"><div style="font:700 9px Consolas,monospace;color:var(--muted);text-transform:uppercase">Side ${i+1}</div><div style="font:800 20px Consolas,monospace;margin-top:3px">${pos.mm} mm</div></div>`).join('')}`;
}

function deletePart(name){
  openActionConfirm('Delete recipe',`Delete "${name}"?`,()=>{
    fetch(`/api/parts/${encodeURIComponent(name)}`,{method:'DELETE'})
      .then(()=>{if(selPart===name){selPart=null;document.getElementById('part-select').value='';onPartSelect('');}loadParts();showToast(`Recipe "${name}" deleted`,'good');})
      .catch(()=>showToast('Delete request failed','err'));
  },{confirmText:'DELETE'});
}

// ── Part editor: mandatory side count + mandatory height for each side ─────
function openEditor(name){
  document.getElementById('modal-title').textContent=name?`Edit - ${name}`:'New part';
  document.getElementById('edit-name').value=name||'';
  document.getElementById('edit-name').disabled=!!name;
  document.getElementById('edit-sides').value=1;
  document.getElementById('steps-wrap').innerHTML='';
  if(name){
    fetch(`/api/parts/${encodeURIComponent(name)}`).then(r=>r.json()).then(p=>{
      document.getElementById('edit-sides').value=p.sides||Math.max(1,(p.positions||[]).length);
      renderSideRows(p.positions||[]);
    });
  }else renderSideRows();
  document.getElementById('modal').classList.add('open');
}
function closeModal(){document.getElementById('modal').classList.remove('open');}

function renderSideRows(existing=null){
  const n=Math.max(1,Math.min(64,parseInt(document.getElementById('edit-sides').value)||1));
  const wrap=document.getElementById('steps-wrap');
  const old=[...wrap.querySelectorAll('.side-mm')].map(i=>i.value);
  const vals=existing?existing.map(x=>x.mm):old;
  wrap.innerHTML='';
  for(let i=0;i<n;i++){
    const tr=document.createElement('tr');
    tr.innerHTML=`<td>SIDE ${i+1}</td><td><input class="side-mm" type="number" min="${PART_HEIGHT_MIN}" max="${PART_HEIGHT_MAX}" step="0.1" value="${vals[i]??''}" placeholder="Required height, 60-400 mm"></td>`;
    wrap.appendChild(tr);
  }
}

function savePart(){
  const name=document.getElementById('edit-name').value.trim();
  const sides=parseInt(document.getElementById('edit-sides').value);
  if(!name){showToast('Part name is required','err');return;}
  if(!Number.isInteger(sides)||sides<1){showToast('Number of sides is required','err');return;}
  const inputs=[...document.querySelectorAll('.side-mm')];
  if(inputs.length!==sides){showToast('Configuration error: side count does not match height rows','err');return;}
  const positions=[];
  for(let i=0;i<sides;i++){
    const raw=inputs[i].value.trim();
    if(raw===''){showToast(`Height for side ${i+1} is required`,'err');return;}
    const mm=parseFloat(raw);
    if(!Number.isFinite(mm)||mm<PART_HEIGHT_MIN||mm>PART_HEIGHT_MAX){showToast(`Side ${i+1} height must be ${PART_HEIGHT_MIN}-${PART_HEIGHT_MAX} mm`,'err');return;}
    positions.push({mm,label:`Side ${i+1}`});
  }
  fetch('/api/parts',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,sides,positions})})
    .then(r=>r.json()).then(d=>{if(d.ok){closeModal();loadParts();document.getElementById('part-select').value=name;onPartSelect(name);showToast('Part recipe saved','good');}else showToast(d.error||'Save failed','err');}).catch(()=>showToast('Save request failed','err'));
}

// ── Homing ────────────────────────────────────────────────────────────────
function startHoming(){ fetch('/api/home/start',{method:'POST'}); }
function stopHoming(){  fetch('/api/home/stop', {method:'POST'}); }
function saveHomingSettings(){
  fetch('/api/home/settings',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({
      speed_fast:parseInt(document.getElementById('h-fast').value),
      speed_slow:parseInt(document.getElementById('h-slow').value),
      timeout:   parseInt(document.getElementById('h-timeout').value),
    })
  }).then(r=>r.json()).then(d=>{if(d.ok)showToast('Homing settings saved','good');else showToast(d.error||'Save failed','err');}).catch(()=>showToast('Save request failed','err'));
}
fetch('/api/home/settings').then(r=>r.json()).then(d=>{
  document.getElementById('h-fast').value   =d.speed_fast;
  document.getElementById('h-slow').value   =d.speed_slow;
  document.getElementById('h-timeout').value=d.timeout;
});

// ── Laser settings ────────────────────────────────────────────────────────
function saveLaserSettings(){
  const pulse_ms=parseInt(document.getElementById('pulse-ms').value);
  if(isNaN(pulse_ms)||pulse_ms<50||pulse_ms>2000){
    showToast('Pulse duration must be 50–2000ms','err');return;
  }
  fetch('/api/settings/laser',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({pulse_ms})
  }).then(r=>r.json()).then(d=>{if(d.ok)showToast('Laser settings saved','good');else showToast(d.error||'Save failed','err');}).catch(()=>showToast('Save request failed','err'));
}
fetch('/api/settings/laser').then(r=>r.json()).then(d=>{
  document.getElementById('pulse-ms').value=d.pulse_ms||100;
});

// ── Jog ───────────────────────────────────────────────────────────────────
function doJog(direction){
  const dist=parseFloat(document.getElementById('jog-dist').value);
  if(isNaN(dist)||dist<=0||dist>400){
    showToast('Distance must be 0.1–400mm','err');return;
  }
  fetch('/api/jog',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({direction, distance_mm:dist})
  });
}

// ── Manual Laser Program ─────────────────────────────────────────────────
async function manualCall(url,body={}){
  try{
    const r=await fetchWithTimeout(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)},5000);
    let d={};
    try{d=await r.json();}catch(_e){}
    if(!r.ok||!d.ok){showToast(d.error||'Command rejected','err');return false;}
    poll();
    return true;
  }catch(e){
    showToast(e&&e.name==='AbortError'?'Controller request timed out':'Controller request failed','err');
    return false;
  }
}
function manualProgramStart(){
  const p=document.getElementById('part-select').value;
  if(!p){showToast('Select a part first','warn');return;}
  manualCall('/api/manual/start',{part:p});
}
function manualMark(){manualCall('/api/manual/mark');}
function manualProgramStop(){manualCall('/api/manual/stop');}
function manualProgramResume(){manualCall('/api/manual/resume');}
function manualProgramReset(){
  if(lastStatus && lastStatus.manual_awaiting_result){
    if(!lastStatus.manual_stopped){
      showToast('A laser result is still outstanding. Press STOP first.','warn');
      return;
    }
    openActionConfirm(
      'Recover Laser Program',
      'EzCad2 did not send the expected OUT4/OUT5 signal. Confirm that EzCad2 marking has stopped.',
      ()=>manualCall('/api/manual/reset',{confirm_abort:true}),
      {confirmText:'RESET PROGRAM',warning:'RESET discards the pending laser result, clears the side count, and moves Z back to Side 1.'}
    );
    return;
  }
  manualCall('/api/manual/reset');
}

// ── Auto ──────────────────────────────────────────────────────────────────
function startAuto(){
  const p=document.getElementById('part-select').value;
  if(!p){showToast('Select a part first','warn');return;}
  const n=parseInt(document.getElementById('cycle-count').value)||1;
  fetch('/api/auto/start',{
    method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({part:p,cycles:n})
  });
}
function pauseResume(){ fetch('/api/auto/pause',{method:'POST'}); }

// ── Stop ──────────────────────────────────────────────────────────────────
function doStop(){ fetch('/api/stop',{method:'POST'}); }

// ── Utils ─────────────────────────────────────────────────────────────────
function esc(s){
  return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');
}
function ea(s){ return esc(s); }
</script>
</body>
</html>"""

# ═════════════════════════════════════════════════════════════════════════════
# ROUTES
# ═════════════════════════════════════════════════════════════════════════════

@app.route("/")
def index():
    appearance = controller.settings.get_section("appearance") or {}
    theme = appearance.get("theme", "light")
    return render_template_string(
        HTML,
        max_travel=MAX_TRAVEL_MM,
        part_height_min=PART_HEIGHT_MIN_MM,
        part_height_max=PART_HEIGHT_MAX_MM,
        theme=theme,
    )

@app.route("/api/status")
def api_status():
    return jsonify(controller.get_status())

@app.route("/api/signals")
def api_signals():
    return jsonify(controller.gpio.read_all())

@app.route("/api/logs")
def api_logs():
    return jsonify({"logs": controller.get_logs()})

@app.route("/api/logs/clear", methods=["POST"])
def api_logs_clear():
    with controller._lock:
        controller.log_lines.clear()
    return jsonify({"ok": True})

@app.route("/api/home/start", methods=["POST"])
def api_home_start():
    return jsonify({"ok": controller.start_homing()})

@app.route("/api/home/stop", methods=["POST"])
def api_home_stop():
    controller.stop_homing()
    return jsonify({"ok": True})

@app.route("/api/home/settings", methods=["GET"])
def api_home_settings_get():
    return jsonify(controller.settings.get_section("homing"))

@app.route("/api/home/settings", methods=["POST"])
def api_home_settings_save():
    data = request.get_json() or {}
    controller.settings.update_section("homing",
        {k: v for k, v in data.items()
         if k in {"speed_fast", "speed_slow", "timeout"}})
    return jsonify({"ok": True})

@app.route("/api/settings/laser", methods=["GET"])
def api_laser_settings_get():
    return jsonify(controller.settings.get_section("laser"))

@app.route("/api/settings/laser", methods=["POST"])
def api_laser_settings_save():
    data = request.get_json() or {}
    controller.settings.update_section("laser",
        {k: v for k, v in data.items() if k in {"pulse_ms"}})
    return jsonify({"ok": True})

@app.route("/api/settings/appearance", methods=["GET"])
def api_appearance_get():
    return jsonify(controller.settings.get_section("appearance") or {"theme": "light"})

@app.route("/api/settings/theme", methods=["POST"])
def api_theme_save():
    data = request.get_json() or {}
    controller.settings.update_section("appearance",
        {"theme": data.get("theme", "light")})
    return jsonify({"ok": True})

@app.route("/api/mode", methods=["POST"])
def api_mode():
    data = request.get_json() or {}
    controller.set_mode(data.get("mode", "manual"))
    return jsonify({"ok": True})

@app.route("/api/jog", methods=["POST"])
def api_jog():
    data = request.get_json() or {}
    direction = data.get("direction", "up")
    distance = float(data.get("distance_mm", 1.0))
    if not controller.position_ready():
        controller.log("Jog request rejected - trusted absolute machine position unavailable")
        return jsonify({"ok": False, "error": "Trusted absolute machine position unavailable"}), 409
    if controller.manual_program_started:
        return jsonify({"ok": False, "error": "Jog is locked while the manual Laser Program is started"}), 409
    threading.Thread(target=controller.jog, args=(direction, distance), daemon=True).start()
    return jsonify({"ok": True})

@app.route("/api/manual/select", methods=["POST"])
def api_manual_select():
    data = request.get_json() or {}
    ok, err = controller.select_manual_part(data.get("part", ""))
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/manual/start", methods=["POST"])
def api_manual_start():
    data = request.get_json() or {}
    ok, err = controller.start_manual_program(data.get("part", ""))
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/manual/mark", methods=["POST"])
def api_manual_mark():
    ok, err = controller.manual_mark(source="gui")
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/manual/stop", methods=["POST"])
def api_manual_stop():
    ok, err = controller.stop_manual_program()
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/manual/resume", methods=["POST"])
def api_manual_resume():
    ok, err = controller.resume_manual_program()
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/manual/reset", methods=["POST"])
def api_manual_reset():
    data = request.get_json(silent=True) or {}
    ok, err = controller.reset_manual_program(
        confirm_abort=bool(data.get("confirm_abort", False))
    )
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/auto/start", methods=["POST"])
def api_auto_start():
    data = request.get_json() or {}
    ok = controller.start_auto(
        data.get("part", ""), int(data.get("cycles", 1)))
    return jsonify({"ok": ok})

@app.route("/api/auto/pause", methods=["POST"])
def api_auto_pause():
    controller.pause_resume()
    return jsonify({"ok": True})

@app.route("/api/stop", methods=["POST"])
def api_stop():
    controller.stop()
    return jsonify({"ok": True})

@app.route("/api/parts")
def api_parts_list():
    names = controller.parts.list_parts()
    result = []
    for name in names:
        p = controller.parts.load(name)
        valid, err = controller.parts.validate(p)
        result.append({
            "name": name,
            "steps": len(p.get("positions", [])) if p else 0,
            "sides": int(p.get("sides", 0)) if p and str(p.get("sides", "")).isdigit() else 0,
            "valid": valid,
            "error": None if valid else err,
        })
    return jsonify({"parts": result})

@app.route("/api/parts/<name>")
def api_part_get(name):
    p = controller.parts.load(name)
    if not p:
        return jsonify({"error": "Not found"}), 404
    return jsonify(p)

@app.route("/api/parts", methods=["POST"])
def api_part_save():
    data = request.get_json() or {}
    ok, msg = controller.parts.save(data)
    return jsonify({"ok": ok, "error": msg if not ok else None})

@app.route("/api/parts/<name>", methods=["DELETE"])
def api_part_delete(name):
    return jsonify({"ok": controller.parts.delete(name)})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)