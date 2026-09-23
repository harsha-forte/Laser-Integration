"""
laser_app_console.py - alternative cockpit UI for laser marking automation
Run: sudo venv/bin/python3 laser_app_console.py
Open: http://192.168.17.132:5000
"""

from flask import Flask, jsonify, request, render_template_string
from laser_ctrl import (
    controller, AXES, AXIS_CONFIG, MAX_TRAVEL_MM,
    PART_HEIGHT_MIN_MM, PART_HEIGHT_MAX_MM,
)
import threading, atexit, time

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
/* transfer axis: horizontal position bar */
.travel-track.horiz{height:auto;display:flex;flex-direction:column;gap:7px;margin-top:12px}
.travel-track.horiz .track{height:12px;border-left:none;border-bottom:1px solid var(--line);border-radius:4px;background:linear-gradient(to right,rgba(0,215,163,.12),transparent)}
.travel-track.horiz .track:before,.travel-track.horiz .track:after{display:none}
.travel-track.horiz .z-fill{top:4px;bottom:auto;left:0;height:4px;width:0%;background:linear-gradient(to right,var(--accent),var(--accent2));transition:width .25s}
.travel-track.horiz .z-fill:after{top:-4px;left:auto;right:-6px}
.travel-track.horiz .ruler{flex-direction:row;justify-content:space-between;text-align:left}
.travel-track.horiz.home-right .track{background:linear-gradient(to left,rgba(0,215,163,.12),transparent)}
.travel-track.horiz.home-right .z-fill{left:auto;right:0;background:linear-gradient(to left,var(--accent),var(--accent2))}
.travel-track.horiz.home-right .z-fill:after{right:auto;left:-6px}
/* laser axis: short vertical bar */
.travel-track.vert{height:96px;margin-top:10px}
.position-zone+.position-zone{border-top:1px solid var(--line)}
.home-banner{display:none;margin-top:10px;font-size:11px;font-weight:700;line-height:1.45;color:var(--warn);border:1px solid var(--warn);border-radius:8px;padding:8px 10px;cursor:pointer}
.home-banner.show{display:block}
.disabled-note{margin:12px 0 0;font-size:12px;font-weight:700;color:var(--warn);border:1px solid var(--warn);border-radius:9px;padding:10px 12px}
/* two-axis jog */
.jog-axes{flex:1;display:grid;grid-template-columns:1fr 1fr;gap:14px;margin-top:16px}
.jog-card{background:var(--panel2);border:1px solid var(--line);border-radius:12px;padding:14px;display:flex;flex-direction:column;gap:12px}
.jog-card.off .jog-pad{opacity:.45}
.jog-card-head{display:flex;justify-content:space-between;align-items:center}
.jog-card-title{font-size:14px;font-weight:800}
.jog-card-sub{font:9px Consolas,monospace;color:var(--muted);margin-top:3px}
.jog-readout{font:800 30px Consolas,monospace;letter-spacing:-.04em}.jog-readout small{font-size:12px;color:var(--muted)}
.jog-note{font-size:10px;color:var(--muted);min-height:14px}.jog-note.warn{color:var(--warn)}.jog-note.err{color:var(--danger)}
.jog-pad{display:grid;gap:10px;align-items:center}
.jog-pad.horizontal{grid-template-columns:1fr 130px 1fr}
.jog-pad.vertical{grid-template-columns:1fr;justify-items:center}
.jog-pad .jog-arrow{height:92px;width:100%;font-size:40px}
.jog-pad.vertical .jog-arrow{height:70px;width:170px}
.jog-pad label{display:block;font:700 9px Consolas,monospace;color:var(--muted);letter-spacing:.1em;text-transform:uppercase;margin-bottom:6px;text-align:center}
.jog-pad input{text-align:center;font:800 22px Consolas,monospace;padding:10px}
.jog-pad.vertical .jog-dist-box{width:170px}
@media(max-width:1180px){.jog-axes{grid-template-columns:1fr}}
.jog-pad .jog-arrow{touch-action:none;user-select:none;-webkit-user-select:none;-webkit-touch-callout:none}
/* equal-height cards: pad fills the middle, STOP buttons sit on the same bottom line */
.jog-card .jog-pad{flex:1;align-content:center}
.jog-card>.btn-red{margin-top:auto}
.jog-pad .jog-arrow.held{background:var(--accent2);color:#fff;border-color:var(--accent2);transform:none}
</style>
</head>
<body>
<header class="topbar">
  <div class="brand"><div class="brand-symbol"></div><div><h1>Laser Cell Console</h1><p>Transfer + laser axis marking station / operator terminal</p></div></div>
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
            <span id="hb-conn-transfer" class="badge b-err">Transfer: --</span>
            <span id="hb-conn-laser" class="badge b-err">Laser: --</span>
            <span id="hb-state" class="badge b-muted">IDLE</span>
            <div class="home-banner" id="home-banner" onclick="switchPage('settings',document.getElementById('nav-settings'))"></div>
          </div>
          <div class="position-zone">
            <div class="position-label">{{ t.name }} position</div>
            <div class="position-number"><span id="ov-pos-transfer">--</span><small> mm</small></div>
            <div class="travel-track horiz{% if t.home_side == 'right' %} home-right{% endif %}">
              <div class="track"><div class="z-fill" id="fill-transfer"></div></div>
              <div class="ruler">{% if t.home_side == 'right' %}<span>{{ t.max_travel_mm|int }}</span><span>{{ (t.max_travel_mm/2)|int }}</span><span>0 (home)</span>{% else %}<span>0 (home)</span><span>{{ (t.max_travel_mm/2)|int }}</span><span>{{ t.max_travel_mm|int }}</span>{% endif %}</div>
            </div>
          </div>
          <div class="position-zone">
            <div class="position-label">{{ l.name }} position</div>
            <div class="position-number"><span id="ov-pos-laser">--</span><small> mm</small></div>
            <div class="travel-track vert">
              <div class="ruler"><span>{{ l.max_travel_mm|int }}</span><span>{{ (l.max_travel_mm/2)|int }}</span><span>0</span></div>
              <div class="track"><div class="z-fill" id="fill-laser"></div></div>
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
                  <div class="op-copy">START moves the transfer axis to the loading position and the laser axis to the Side 1 height. Load the part, then press MARK or the foot pedal: the transfer axis moves under the laser, both positions are verified and GPIO17 fires. OUT4 moves the laser axis to the next side height and fires again; OUT5 returns to the loading position.</div>
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
                    <div class="sequence-copy">The pedal is accepted only while the transfer axis is at the Side 1 initial position. A pedal press does not re-pulse GPIO17.</div>
                  </div>
                  <div class="sequence-box">
                    <div class="sequence-head"><span>Current recipe</span><span id="manual-step">0 / 0</span></div>
                    <div class="sequence-name" id="manual-summary-title">Select a part</div>
                    <div class="sequence-copy" id="manual-summary-copy">Every side requires a part height above {{ part_height_min }} mm.</div>
                    <div class="progress-track"><span id="manual-progress"></span></div>
                  </div>
                </div>
              </div>
            </div>
          </div>

          <div id="sub-auto" class="subtab-pane">
            <div class="operation">
              <div class="op-top"><div><h2 class="op-title">Automatic Cycle</h2><div class="op-copy">Run repeated laser cycles with robot placement and pickup handshake placeholders.</div></div><span class="mode-chip">sequence mode</span></div>
              <div class="disabled-note auto-disabled-note">Automatic cycle is not redesigned for two axes yet.</div>
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
                    <div class="process-step"><i></i><span>02 / Transfer axis moves to recipe position</span></div>
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
              <div class="op-top"><div><h2 class="op-title">Axis Jog</h2><div class="op-copy">Switch an axis ON, then press and HOLD an arrow: the axis moves while held and stops when released, or after the entered maximum distance. Each axis must be homed first; travel is also limited by the machine range and the group collision rule.</div></div><span class="mode-chip">service motion</span></div>
              <div class="jog-axes">
                <div class="jog-card off" id="jogcard-transfer">
                  <div class="jog-card-head">
                    <div><div class="jog-card-title">{{ t.name }}</div><div class="jog-card-sub">left / right · 0-{{ t.max_travel_mm|int }} mm · {{ t.port }}</div></div>
                    <label class="toggle"><input type="checkbox" id="jogen-transfer" onchange="setJogEnabled('transfer',this)"><span class="slider"></span></label>
                  </div>
                  <div class="jog-readout"><span id="jogpos-transfer">--</span><small> mm</small></div>
                  <div class="jog-note" id="jognote-transfer">--</div>
                  <div class="jog-pad horizontal">
                    <button class="jog-arrow" id="jog-transfer-left" data-axis="transfer" data-dir="left" title="Hold to jog left">&#9664;</button>
                    <div><label>Max distance / mm (0.1–{{ t.max_travel_mm|int }})</label><input type="number" id="jogdist-transfer" min="0.1" max="{{ t.max_travel_mm }}" step="0.1" value="1"></div>
                    <button class="jog-arrow" id="jog-transfer-right" data-axis="transfer" data-dir="right" title="Hold to jog right">&#9654;</button>
                  </div>
                  <button class="btn btn-red full" onclick="stopAxis('transfer')">STOP TRANSFER AXIS</button>
                </div>
                <div class="jog-card off" id="jogcard-laser">
                  <div class="jog-card-head">
                    <div><div class="jog-card-title">{{ l.name }}</div><div class="jog-card-sub">up / down · 0-{{ l.max_travel_mm|int }} mm · {{ l.port }}</div></div>
                    <label class="toggle"><input type="checkbox" id="jogen-laser" onchange="setJogEnabled('laser',this)"><span class="slider"></span></label>
                  </div>
                  <div class="jog-readout"><span id="jogpos-laser">--</span><small> mm</small></div>
                  <div class="jog-note" id="jognote-laser">--</div>
                  <div class="jog-pad vertical">
                    <button class="jog-arrow" id="jog-laser-up" data-axis="laser" data-dir="up" title="Hold to jog up">&#9650;</button>
                    <div class="jog-dist-box"><label>Max distance / mm (0.1–{{ l.max_travel_mm|int }})</label><input type="number" id="jogdist-laser" min="0.1" max="{{ l.max_travel_mm }}" step="0.1" value="1"></div>
                    <button class="jog-arrow" id="jog-laser-down" data-axis="laser" data-dir="down" title="Hold to jog down">&#9660;</button>
                  </div>
                  <button class="btn btn-red full" onclick="stopAxis('laser')">STOP LASER AXIS</button>
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
            <div class="motor-stat"><div class="motor-label">Transfer pos</div><div class="motor-value"><span id="m-pos-transfer">--</span><span class="motor-unit">mm</span></div></div>
            <div class="motor-stat"><div class="motor-label">Transfer speed</div><div class="motor-value"><span id="m-spd-transfer">--</span><span class="motor-unit">rpm</span></div></div>
            <div class="motor-stat"><div class="motor-label">Laser pos</div><div class="motor-value"><span id="m-pos-laser">--</span><span class="motor-unit">mm</span></div></div>
            <div class="motor-stat"><div class="motor-label">Laser speed</div><div class="motor-value"><span id="m-spd-laser">--</span><span class="motor-unit">rpm</span></div></div>
            <div class="motor-stat wide"><div class="motor-label">Controller state</div><div class="motor-value" id="m-state" style="font-size:13px">--</div></div>
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
              <div class="settings-title">{{ t.name }} homing</div>
              <div class="homing-status"><span class="dot" id="home-dot-transfer"></span><span id="home-text-transfer">--</span></div>
              <div class="form-grid">
                <div><label>Fast search speed / rpm (5–200)</label><input type="number" id="h-fast-transfer" min="5" max="200" step="5"></div>
                <div><label>Slow approach / rpm (1–50)</label><input type="number" id="h-slow-transfer" min="1" max="50" step="1"></div>
                <div class="span2"><label>Homing timeout / seconds (10–600)</label><input type="number" id="h-timeout-transfer" min="10" max="600" step="10"></div>
              </div>
              <div class="help">Method 17: moves to the NL switch. Home is the switch release point (0 mm); the axis does not move further after homing.</div>
              <div class="btn2"><button class="btn btn-amber" id="btn-home-transfer" onclick="startHoming('transfer')">START HOMING</button><button class="btn btn-red" id="btn-hstop-transfer" onclick="stopHoming('transfer')">STOP HOMING</button></div>
              <button class="btn btn-ghost full mt8" onclick="saveHomingSettings('transfer')">SAVE HOMING PARAMETERS</button>
            </div>
            <div class="settings-card">
              <div class="settings-title">{{ l.name }} homing</div>
              <div class="homing-status"><span class="dot" id="home-dot-laser"></span><span id="home-text-laser">--</span></div>
              <div class="form-grid">
                <div><label>Fast search speed / rpm (5–200)</label><input type="number" id="h-fast-laser" min="5" max="200" step="5"></div>
                <div><label>Slow approach / rpm (1–50)</label><input type="number" id="h-slow-laser" min="1" max="50" step="1"></div>
                <div class="span2"><label>Homing timeout / seconds (10–600)</label><input type="number" id="h-timeout-laser" min="10" max="600" step="10"></div>
              </div>
              <div class="help">Method 17: moves down to the NL switch, then moves 2.5 mm up to 0 mm.</div>
              <div class="btn2"><button class="btn btn-amber" id="btn-home-laser" onclick="startHoming('laser')">START HOMING</button><button class="btn btn-red" id="btn-hstop-laser" onclick="stopHoming('laser')">STOP HOMING</button></div>
              <button class="btn btn-ghost full mt8" onclick="saveHomingSettings('laser')">SAVE HOMING PARAMETERS</button>
            </div>
            <div class="settings-card">
              <div class="settings-title">Manual program</div>
              <div class="form-grid">
                <div><label>Loading position / mm (0–{{ t.max_travel_mm|int }})</label><input type="number" id="pg-load" min="0" max="{{ t.max_travel_mm }}" step="0.1"></div>
                <div><label>Laser position / mm (0–{{ t.max_travel_mm|int }})</label><input type="number" id="pg-laser" min="0" max="{{ t.max_travel_mm }}" step="0.1"></div>
                <div><label>Delay after OUT5 / s (0–60)</label><input type="number" id="pg-delay" min="0" max="60" step="0.1"></div>
                <div><label>OUT4/OUT5 timeout / s (5–1800)</label><input type="number" id="pg-timeout" min="5" max="1800" step="5"></div>
                <div><label>OUT5-after-OUT4 timeout / s (1–120)</label><input type="number" id="pg-ctimeout" min="1" max="120" step="1"></div>
                <div><label>Transfer speed / rpm (10–3000)</label><input type="number" id="pg-trpm" min="10" max="3000" step="10"></div>
                <div><label>Laser speed / rpm (10–3000)</label><input type="number" id="pg-lrpm" min="10" max="3000" step="10"></div>
                <div class="span2"><label>Speed override / % (10–100) — scales both program speeds</label><input type="number" id="pg-ovr" min="10" max="100" step="5"></div>
              </div>
              <div class="help">Both transfer positions must respect the collision rule together with the part heights. Part heights below {{ part_height_min }} mm are rejected when a recipe is saved.</div>
              <button class="btn btn-ghost full" onclick="saveProgramSettings()">SAVE PROGRAM PARAMETERS</button>
            </div>
            <div class="settings-card">
              <div class="settings-title">Group collision configuration</div>
              <div class="toggle-row" style="margin-bottom:12px"><div><div class="toggle-label">Collision rule active</div><div class="toggle-copy">Applies to jog, homing, manual and automatic moves</div></div>
                <label class="toggle"><input type="checkbox" id="col-enabled"><span class="slider"></span></label></div>
              <div class="form-grid">
                <div><label>Transfer axis limit / mm (0–{{ t.max_travel_mm|int }})</label><input type="number" id="col-transfer" min="0" max="{{ t.max_travel_mm }}" step="0.1"></div>
                <div><label>Laser axis minimum / mm (0–{{ l.max_travel_mm|int }})</label><input type="number" id="col-laser" min="0" max="{{ l.max_travel_mm }}" step="0.1"></div>
              </div>
              <div class="help" id="col-help">While the transfer axis is above its limit, the laser axis may not be below its minimum. Moves that would break the rule are blocked; jog moves stop at the limit. Unhomed or disconnected axes are treated as being in the worst position, so home the transfer axis before the laser axis.</div>
              <div class="homing-status" id="col-status"><span class="dot" id="col-dot"></span><span id="col-text">--</span></div>
              <button class="btn btn-ghost full" onclick="saveCollision()">SAVE COLLISION PARAMETERS</button>
            </div>
            <div class="settings-card">
              <div class="settings-title">Laser trigger</div>
              <label>Laser trigger pulse / ms (50–2000)</label><input type="number" id="pulse-ms" min="50" max="2000" step="50" value="100">
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

  <div class="log-wrap"><div class="log-hdr"><span class="log-title">Machine event terminal</span><button class="log-clear" onclick="copyLog()">COPY</button><button class="log-clear" onclick="downloadLog()">DOWNLOAD</button><button class="log-clear" onclick="clearLog()">CLEAR BUFFER</button></div><div class="log-box" id="log-box"></div></div>
</div>

<div class="overlay" id="modal"><div class="modal">
  <h2 id="modal-title">New part</h2>
  <label>Part name</label><input type="text" id="edit-name" placeholder="e.g. Part_A">
  <div style="margin-top:12px"><label>Number of sides to be lasered</label><input type="number" id="edit-sides" min="1" max="64" step="1" value="1" onchange="renderSideRows()" oninput="renderSideRows()"></div>
  <div class="help">Required: every side must have an axis position. Different sides may use the same position.</div>
  <table class="side-table"><thead><tr><th>Side</th><th>Part height (laser axis) / mm ({{ part_height_min|int }}–{{ part_height_max|int }})</th></tr></thead><tbody id="steps-wrap"></tbody></table>
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
const AXES_CFG = {{ axes_cfg|tojson }};
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
  const AX=d.axes||{};
  for(const k of ['transfer','laser']){
    const a=AX[k]; if(!a) continue;
    const b=document.getElementById('hb-conn-'+k);
    const label=k==='transfer'?'Transfer':'Laser';
    let cls='b-err',txt=`${label}: not connected`;
    if(a.connected){
      if(a.busy==='homing'){cls='b-warn';txt=`${label}: homing...`;}
      else if(a.homed){cls='b-ok';txt=`${label}: ready`;}
      else {cls='b-warn';txt=`${label}: homing required`;}
    }
    b.className='badge '+cls; b.textContent=txt; b.title=a.error||a.port;
  }
  const banner=document.getElementById('home-banner');
  const need=d.homing_suggested||[];
  banner.classList.toggle('show',need.length>0);
  banner.textContent=need.length?`Homing required: ${need.join(', ')} - open Configure to home.`:'';
  const sb=document.getElementById('hb-state');
  sb.textContent=d.state; sb.style.color=SC[d.state]||'var(--muted)';

  const fmt=v=>(v===null||v===undefined)?'--':Number(v).toFixed(2);
  for(const k of ['transfer','laser']){
    const a=AX[k]||{};
    document.getElementById('m-pos-'+k).textContent=fmt(a.position);
    document.getElementById('m-spd-'+k).textContent=(a.speed===null||a.speed===undefined)?'--':a.speed;
    document.getElementById('ov-pos-'+k).textContent=fmt(a.position);
    const f=document.getElementById('fill-'+k);
    if(f){const pct=(a.position!==null&&a.position!==undefined)?Math.max(0,Math.min(100,(a.position/a.max_travel_mm)*100)):0;
      if(k==='transfer') f.style.width=pct+'%'; else f.style.height=pct+'%';}
    updateJogCard(k,a,d.collision);
    updateHomingCard(k,a);
  }
  document.getElementById('m-state').textContent=d.state||'--';
  document.getElementById('m-state').style.color=SC[d.state]||'var(--muted)';

  const ovs=document.getElementById('ov-state'); if(ovs){ovs.textContent=d.state||'--';ovs.style.color=SC[d.state]||'var(--text)';}
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
    else if(d.manual_stopped){msg.textContent='Program stopped. Both axes stand where they are. RESUME continues the interrupted step.';msg.classList.add('warn');}
    else if(d.manual_mark_enabled){msg.textContent='At the loading position. Load the part, then press MARK or the foot pedal.';msg.classList.add('good');}
    else if(d.manual_awaiting_complete){msg.textContent='Last side reported OUT4. Waiting for OUT5 to confirm the part is complete.';msg.classList.add('warn');}
    else if(d.manual_awaiting_result){msg.textContent=`Side ${side}/${total} is active. Waiting for ${side<total?'OUT4 (side finished)':'OUT4 then OUT5 (part complete)'}.`;msg.classList.add('warn');}
    else if(['MOVING_TO_LOAD','MOVING_TO_LASER','MOVING_NEXT_SIDE','RETURNING_LOAD','RESETTING'].includes(d.manual_program_state)){msg.textContent='Axes are moving. Operator start is locked.';msg.classList.add('warn');}
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
  const bothReady=!!(d.axes&&d.axes.transfer.ready&&d.axes.laser.ready);
  if(startBtn) startBtn.disabled=!hasPart||!bothReady||d.manual_program_started;
  if(stopBtn) stopBtn.disabled=!d.manual_program_started||d.manual_stopped;
  if(resumeBtn) resumeBtn.disabled=!d.manual_program_started||!d.manual_stopped;
  // During a missing-OUT4/OUT5 recovery, RESET is intentionally enabled
  // only after STOP. This prevents axis motion while EzCad2 may still be active.
  if(resetBtn) resetBtn.disabled=!d.manual_program_started||(d.manual_awaiting_result&&!d.manual_stopped);

  updateCollisionStatus(d.collision);
  // Automatic cycle is not redesigned for two axes yet
  document.querySelectorAll('.auto-disabled-note').forEach(n=>n.style.display=d.auto_enabled?'none':'block');
  if(!d.auto_enabled){
    [document.getElementById('btn-auto-start'),document.getElementById('btn-pause')]
      .forEach(b=>{if(b) b.disabled=true;});
  }
}

function updateHomingCard(k,a){
  const dot=document.getElementById('home-dot-'+k),txt=document.getElementById('home-text-'+k);
  const bh=document.getElementById('btn-home-'+k),bs=document.getElementById('btn-hstop-'+k);
  if(!dot) return;
  if(!a.connected){dot.className='dot lo';dot.style.background='';txt.textContent=a.error||'Drive not connected';}
  else if(a.busy==='homing'){dot.className='dot hi';dot.style.background='var(--warn)';txt.textContent='Homing in progress...';}
  else if(a.homed){dot.className='dot hi';dot.style.background='var(--accent)';txt.textContent=`Homed · position ${a.position!==null?Number(a.position).toFixed(2)+' mm':'--'}`;}
  else{dot.className='dot hi';dot.style.background='var(--warn)';txt.textContent=a.error||'Homing required';}
  bh.disabled=!a.connected||!!a.busy||(lastStatus&&lastStatus.manual_program_started);
  bs.disabled=a.busy!=='homing';
}

function updateJogCard(k,a,col){
  const card=document.getElementById('jogcard-'+k); if(!card) return;
  const tog=document.getElementById('jogen-'+k);
  if(document.activeElement!==tog) tog.checked=!!a.jog_enabled;
  tog.disabled=!a.connected&&!a.jog_enabled;
  document.getElementById('jogpos-'+k).textContent=(a.position===null||a.position===undefined)?'--':Number(a.position).toFixed(2);
  const note=document.getElementById('jognote-'+k);
  let n='',cls='jog-note';
  if(!a.connected){n=a.error||'Drive not connected';cls+=' err';}
  else if(!a.homed){n=a.busy==='homing'?'Homing in progress...':'Not homed - home in Configure before jogging';cls+=' warn';}
  else if(!a.jog_enabled){n='Switch ON to unlock jog';}
  else if(a.busy){n=a.busy==='jog'?'Moving...':`Busy (${a.busy})`;cls+=' warn';}
  else{n='Ready to jog';
    if(col&&col.enabled){
      if(k==='laser'&&col.laser_restricted){n=`Collision rule: laser limited to ≥ ${col.laser_min_mm} mm`;cls+=' warn';}
      if(k==='transfer'&&col.transfer_restricted){n=`Collision rule: transfer limited to ≤ ${col.transfer_limit_mm} mm`;cls+=' warn';}
    }}
  note.className=cls; note.textContent=n;
  // NOTE: disabling a button that holds the pointer capture makes the browser
  // fire lostpointercapture, which would cancel the jog the user is holding.
  const holding=!!jogState[k];
  const canJog=a.connected&&a.homed&&a.jog_enabled&&!a.busy;
  card.classList.toggle('off',!a.jog_enabled);
  const btns=k==='transfer'?['left','right']:['up','down'];
  btns.forEach(b=>{const el=document.getElementById(`jog-${k}-${b}`);
    if(el){const dis=!canJog&&!holding; if(el.disabled!==dis) el.disabled=dis;}});
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
let lastLogs=[];
function updateLog(lines){
  lastLogs=lines;
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
function logText(){ return (lastLogs||[]).join(String.fromCharCode(10)); }
async function copyLog(){
  const txt=logText();
  try{ await navigator.clipboard.writeText(txt); showToast('Log copied to clipboard','good'); }
  catch(e){
    const ta=document.createElement('textarea'); ta.value=txt; ta.style.position='fixed'; ta.style.opacity='0';
    document.body.appendChild(ta); ta.select();
    try{ document.execCommand('copy'); showToast('Log copied to clipboard','good'); }
    catch(_e){ showToast('Copy not allowed by the browser - use DOWNLOAD','err'); }
    document.body.removeChild(ta);
  }
}
function downloadLog(){ window.location='/api/logs/download'; }

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
    document.getElementById('manual-summary-copy').textContent=`Every side requires a part height above ${PART_HEIGHT_MIN} mm.`;
    fetch('/api/manual/select',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({part:''})});
    return;
  }
  fetch(`/api/parts/${encodeURIComponent(name)}`).then(r=>r.json()).then(p=>{
    const positions=Array.isArray(p.positions)?p.positions:[];
    const sides=Number(p.sides||0);
    const valid=sides>=1&&positions.length===sides&&positions.every(x=>x.mm!==undefined&&x.mm!==null&&x.mm!=='');
    if(!valid){
      const err=`Part configuration incomplete: a side count and one position for every side are mandatory.`;
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

// ── Part editor: mandatory side count + mandatory position for each side ─────
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
    tr.innerHTML=`<td>SIDE ${i+1}</td><td><input class="side-mm" type="number" min="${PART_HEIGHT_MIN}" max="${PART_HEIGHT_MAX}" step="0.1" value="${vals[i]??''}" placeholder="Part height, ${PART_HEIGHT_MIN}-${PART_HEIGHT_MAX} mm"></td>`;
    wrap.appendChild(tr);
  }
}

function savePart(){
  const name=document.getElementById('edit-name').value.trim();
  const sides=parseInt(document.getElementById('edit-sides').value);
  if(!name){showToast('Part name is required','err');return;}
  if(!Number.isInteger(sides)||sides<1){showToast('Number of sides is required','err');return;}
  const inputs=[...document.querySelectorAll('.side-mm')];
  if(inputs.length!==sides){showToast('Configuration error: side count does not match position rows','err');return;}
  const positions=[];
  for(let i=0;i<sides;i++){
    const raw=inputs[i].value.trim();
    if(raw===''){showToast(`Height for side ${i+1} is required`,'err');return;}
    const mm=parseFloat(raw);
    if(!Number.isFinite(mm)){showToast(`Side ${i+1}: enter a part height`,'err');return;}
    if(mm<PART_HEIGHT_MIN){showToast(`Side ${i+1}: part height should be above ${PART_HEIGHT_MIN} mm`,'err');return;}
    if(mm>PART_HEIGHT_MAX){showToast(`Side ${i+1}: part height must be at most ${PART_HEIGHT_MAX} mm`,'err');return;}
    positions.push({mm,label:`Side ${i+1}`});
  }
  fetch('/api/parts',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({name,sides,positions})})
    .then(r=>r.json()).then(d=>{if(d.ok){closeModal();loadParts();document.getElementById('part-select').value=name;onPartSelect(name);showToast('Part recipe saved','good');}else showToast(d.error||'Save failed','err');}).catch(()=>showToast('Save request failed','err'));
}

// ── Homing (per axis) ─────────────────────────────────────────────────────
async function axisCall(url,body){
  try{
    const r=await fetchWithTimeout(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)},4000);
    let d={}; try{d=await r.json();}catch(_e){}
    if(!r.ok||!d.ok){showToast(d.error||'Command rejected','err');return false;}
    poll(); return true;
  }catch(e){showToast('Controller request failed','err');return false;}
}
function startHoming(axis){ axisCall('/api/home/start',{axis}); }
function stopHoming(axis){  axisCall('/api/home/stop',{axis}); }
function saveHomingSettings(axis){
  const v=id=>parseInt(document.getElementById(`${id}-${axis}`).value);
  const body={axis,speed_fast:v('h-fast'),speed_slow:v('h-slow'),timeout:v('h-timeout')};
  if([body.speed_fast,body.speed_slow,body.timeout].some(x=>!Number.isFinite(x)||x<=0)){showToast('Enter valid homing values','err');return;}
  axisCall('/api/home/settings',body).then(ok=>{if(ok)showToast(`${AXES_CFG[axis].name} homing settings saved`,'good');});
}
for(const axis of ['transfer','laser']){
  fetch(`/api/home/settings?axis=${axis}`).then(r=>r.json()).then(d=>{
    document.getElementById(`h-fast-${axis}`).value   =d.speed_fast;
    document.getElementById(`h-slow-${axis}`).value   =d.speed_slow;
    document.getElementById(`h-timeout-${axis}`).value=d.timeout;
  }).catch(()=>{});
}

// ── Manual program settings ───────────────────────────────────────────────
const PG_FIELDS={'pg-load':'loading_mm','pg-laser':'laser_mm','pg-delay':'out5_delay_s','pg-timeout':'result_timeout_s','pg-ctimeout':'complete_timeout_s','pg-trpm':'transfer_speed_rpm','pg-lrpm':'laser_speed_rpm','pg-ovr':'speed_override_pct'};
function loadProgramSettings(){
  fetch('/api/settings/program').then(r=>r.json()).then(c=>{
    for(const [id,key] of Object.entries(PG_FIELDS)) document.getElementById(id).value=c[key];
  }).catch(()=>{});
}
function saveProgramSettings(){
  const body={};
  for(const [id,key] of Object.entries(PG_FIELDS)){
    const v=parseFloat(document.getElementById(id).value);
    if(!Number.isFinite(v)){showToast('Enter valid program values','err');return;}
    body[key]=v;
  }
  axisCall('/api/settings/program',body).then(ok=>{if(ok){showToast('Program parameters saved','good');loadProgramSettings();}});
}
loadProgramSettings();

// ── Group collision configuration ─────────────────────────────────────────
function loadCollision(){
  fetch('/api/settings/collision').then(r=>r.json()).then(c=>{
    document.getElementById('col-enabled').checked=!!c.enabled;
    document.getElementById('col-transfer').value=c.transfer_limit_mm;
    document.getElementById('col-laser').value=c.laser_min_mm;
  }).catch(()=>{});
}
function saveCollision(){
  const body={
    enabled:document.getElementById('col-enabled').checked,
    transfer_limit_mm:parseFloat(document.getElementById('col-transfer').value),
    laser_min_mm:parseFloat(document.getElementById('col-laser').value),
  };
  if(!Number.isFinite(body.transfer_limit_mm)||!Number.isFinite(body.laser_min_mm)){showToast('Enter both collision limits','err');return;}
  axisCall('/api/settings/collision',body).then(ok=>{if(ok){showToast('Collision configuration saved','good');loadCollision();}});
}
loadCollision();
function updateCollisionStatus(c){
  const dot=document.getElementById('col-dot'),txt=document.getElementById('col-text'); if(!dot||!c) return;
  if(!c.enabled){dot.className='dot lo';dot.style.background='';txt.textContent='Rule OFF - no collision protection';return;}
  dot.className='dot hi';
  if(c.laser_restricted||c.transfer_restricted){
    dot.style.background='var(--warn)';
    const parts=[];
    if(c.laser_restricted) parts.push(`laser limited to ≥ ${c.laser_min_mm} mm`);
    if(c.transfer_restricted) parts.push(`transfer limited to ≤ ${c.transfer_limit_mm} mm`);
    txt.textContent='Active now: '+parts.join(', ');
  } else {dot.style.background='var(--accent)';txt.textContent='Active - no restriction at current positions';}
}

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

// ── Jog (per axis) ────────────────────────────────────────────────────────
function setJogEnabled(axis,el){
  axisCall('/api/axis/enable',{axis,on:el.checked}).then(ok=>{if(!ok) el.checked=!el.checked;});
}
function stopAxis(axis){ axisCall('/api/axis/stop',{axis}); }
// Hold-to-run jog: move while held, stop on release; heartbeat = dead-man
const jogState={transfer:null,laser:null};
function jogBody(o){return {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(o)};}
function jogPress(axis,direction,ev){
  ev.preventDefault();
  if(jogState[axis]) return;
  const btn=ev.currentTarget; if(btn.disabled) return;
  const max=AXES_CFG[axis].max_travel_mm;
  const dist=parseFloat(document.getElementById('jogdist-'+axis).value);
  if(!Number.isFinite(dist)||dist<=0||dist>max){showToast(`Distance must be 0.1–${max} mm`,'err');return;}
  try{btn.setPointerCapture(ev.pointerId);}catch(_e){}
  const st={id:null,timer:null,released:false,btn};
  jogState[axis]=st; btn.classList.add('held');
  fetchWithTimeout('/api/jog',jogBody({axis,direction,distance_mm:dist}),3000)
    .then(async r=>{let d={};try{d=await r.json();}catch(_e){} return {ok:r.ok&&d.ok,d};})
    .then(({ok,d})=>{
      if(jogState[axis]!==st) return;
      if(!ok){showToast(d.error||'Jog rejected','err');jogEnd(axis);return;}
      st.id=d.jog_id;
      if(st.released){sendJogRelease(axis,st.id);jogEnd(axis);return;}
      st.timer=setInterval(()=>{fetch('/api/jog/hold',jogBody({axis,id:st.id})).catch(()=>{});},150);
    })
    .catch(()=>{showToast('Jog request failed','err');jogEnd(axis);});
}
function jogRelease(axis){
  const st=jogState[axis]; if(!st) return;
  if(st.id===null){st.released=true;return;}      // start still in flight
  sendJogRelease(axis,st.id); jogEnd(axis);
}
function sendJogRelease(axis,id){fetch('/api/jog/release',jogBody({axis,id})).catch(()=>{});}
function jogEnd(axis){
  const st=jogState[axis]; if(!st) return;
  if(st.timer) clearInterval(st.timer);
  st.btn.classList.remove('held'); jogState[axis]=null;
}
document.querySelectorAll('.jog-pad .jog-arrow[data-axis]').forEach(btn=>{
  const axis=btn.dataset.axis,dir=btn.dataset.dir;
  btn.addEventListener('pointerdown',e=>jogPress(axis,dir,e));
  ['pointerup','pointercancel'].forEach(t=>btn.addEventListener(t,()=>jogRelease(axis)));
  btn.addEventListener('pointerleave',e=>{if(!btn.hasPointerCapture||!btn.hasPointerCapture(e.pointerId)) jogRelease(axis);});
  btn.addEventListener('contextmenu',e=>e.preventDefault());
});
window.addEventListener('blur',()=>{jogRelease('transfer');jogRelease('laser');});
document.addEventListener('visibilitychange',()=>{if(document.hidden){jogRelease('transfer');jogRelease('laser');}});

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
      {confirmText:'RESET PROGRAM',warning:'RESET discards the pending laser result, stops both axes and ends the run. Press LASER PROGRAM START to begin again.'}
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
  axisCall('/api/auto/start',{part:p,cycles:n});
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
        t=AXIS_CONFIG["transfer"],
        l=AXIS_CONFIG["laser"],
        axes_cfg={k: {"name": AXIS_CONFIG[k]["name"],
                      "max_travel_mm": AXIS_CONFIG[k]["max_travel_mm"],
                      "home_side": AXIS_CONFIG[k]["home_side"]} for k in AXES},
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

@app.route("/api/logs/download")
def api_logs_download():
    from flask import Response
    name = time.strftime("laser-log-%Y%m%d-%H%M%S.txt")
    body = "\n".join(controller.get_logs()) + "\n"
    return Response(body, mimetype="text/plain",
                    headers={"Content-Disposition": f"attachment; filename={name}"})

@app.route("/api/logs/clear", methods=["POST"])
def api_logs_clear():
    with controller._lock:
        controller.log_lines.clear()
    return jsonify({"ok": True})

def _axis_arg(data):
    axis = (data or {}).get("axis", "transfer")
    return axis if axis in AXES else None

@app.route("/api/home/start", methods=["POST"])
def api_home_start():
    axis = _axis_arg(request.get_json(silent=True))
    if not axis:
        return jsonify({"ok": False, "error": "Unknown axis"}), 400
    ok, err = controller.start_homing(axis)
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/home/stop", methods=["POST"])
def api_home_stop():
    axis = _axis_arg(request.get_json(silent=True))
    if not axis:
        return jsonify({"ok": False, "error": "Unknown axis"}), 400
    controller.stop_homing(axis)
    return jsonify({"ok": True})

@app.route("/api/home/settings", methods=["GET"])
def api_home_settings_get():
    axis = _axis_arg(request.args)
    if not axis:
        return jsonify({"error": "Unknown axis"}), 400
    return jsonify(controller.settings.get_section(f"homing_{axis}"))

@app.route("/api/home/settings", methods=["POST"])
def api_home_settings_save():
    data = request.get_json(silent=True) or {}
    axis = _axis_arg(data)
    if not axis:
        return jsonify({"ok": False, "error": "Unknown axis"}), 400
    vals = {}
    limits = {"speed_fast": (5, 200), "speed_slow": (1, 50), "timeout": (10, 600)}
    for k, (lo, hi) in limits.items():
        if k in data:
            try:
                v = int(data[k])
            except (TypeError, ValueError):
                return jsonify({"ok": False, "error": f"{k} must be a number"}), 400
            if not lo <= v <= hi:
                return jsonify({"ok": False, "error": f"{k} must be {lo}-{hi}"}), 400
            vals[k] = v
    controller.settings.update_section(f"homing_{axis}", vals)
    return jsonify({"ok": True})

@app.route("/api/axis/enable", methods=["POST"])
def api_axis_enable():
    data = request.get_json(silent=True) or {}
    axis = _axis_arg(data)
    if not axis:
        return jsonify({"ok": False, "error": "Unknown axis"}), 400
    ok, err = controller.set_jog_enabled(axis, bool(data.get("on")))
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

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

@app.route("/api/axis/stop", methods=["POST"])
def api_axis_stop():
    axis = _axis_arg(request.get_json(silent=True))
    if not axis:
        return jsonify({"ok": False, "error": "Unknown axis"}), 400
    ok, err = controller.stop_axis(axis)
    return jsonify({"ok": ok, "error": err}), (200 if ok else 409)

@app.route("/api/settings/program", methods=["GET"])
def api_program_get():
    return jsonify(controller.settings.get_section("program"))

@app.route("/api/settings/program", methods=["POST"])
def api_program_save():
    data = request.get_json(silent=True) or {}
    t_max = AXIS_CONFIG["transfer"]["max_travel_mm"]
    limits = {
        "loading_mm": (0, t_max), "laser_mm": (0, t_max),
        "out5_delay_s": (0, 60), "result_timeout_s": (5, 1800),
        "complete_timeout_s": (1, 120),
        "transfer_speed_rpm": (10, 3000), "laser_speed_rpm": (10, 3000),
        "speed_override_pct": (10, 100),
    }
    vals = {}
    for k, (lo, hi) in limits.items():
        if k not in data:
            continue
        try:
            v = float(data[k])
        except (TypeError, ValueError):
            return jsonify({"ok": False, "error": f"{k} must be a number"}), 400
        if not lo <= v <= hi:
            return jsonify({"ok": False, "error": f"{k} must be {lo:g}-{hi:g}"}), 400
        vals[k] = int(v) if k.endswith("rpm") or k.endswith("timeout_s") or k.endswith("pct") else v
    if controller.manual_program_started:
        return jsonify({"ok": False, "error": "Stop the Laser Program before changing its parameters"}), 409
    controller.settings.update_section("program", vals)
    controller.log(f"Program parameters saved: loading {vals.get('loading_mm')} mm, "
                   f"laser {vals.get('laser_mm')} mm")
    return jsonify({"ok": True})

@app.route("/api/settings/collision", methods=["GET"])
def api_collision_get():
    return jsonify(controller.settings.get_section("collision"))

@app.route("/api/settings/collision", methods=["POST"])
def api_collision_save():
    data = request.get_json(silent=True) or {}
    try:
        t_lim = float(data.get("transfer_limit_mm"))
        l_min = float(data.get("laser_min_mm"))
    except (TypeError, ValueError):
        return jsonify({"ok": False, "error": "Both limits must be numbers"}), 400
    t_max = AXIS_CONFIG["transfer"]["max_travel_mm"]
    l_max = AXIS_CONFIG["laser"]["max_travel_mm"]
    if not 0 <= t_lim <= t_max:
        return jsonify({"ok": False, "error": f"Transfer limit must be 0-{t_max:g} mm"}), 400
    if not 0 <= l_min <= l_max:
        return jsonify({"ok": False, "error": f"Laser minimum must be 0-{l_max:g} mm"}), 400
    enabled = bool(data.get("enabled", True))
    controller.settings.update_section("collision", {
        "enabled": enabled, "transfer_limit_mm": t_lim, "laser_min_mm": l_min})
    controller.log(f"Group collision rule {'ON' if enabled else 'OFF'}: transfer > {t_lim:g} mm "
                   f"=> laser >= {l_min:g} mm")
    return jsonify({"ok": True})

@app.route("/api/jog", methods=["POST"])
def api_jog():
    """Start a hold-to-run jog; the browser then sends /api/jog/hold heartbeats."""
    data = request.get_json(silent=True) or {}
    axis = _axis_arg(data)
    if not axis:
        return jsonify({"ok": False, "error": "Unknown axis"}), 400
    ok, err, jog_id = controller.jog(axis, data.get("direction", ""), data.get("distance_mm", 1.0))
    return jsonify({"ok": ok, "error": err, "jog_id": jog_id}), (200 if ok else 409)

@app.route("/api/jog/hold", methods=["POST"])
def api_jog_hold():
    data = request.get_json(silent=True) or {}
    axis = _axis_arg(data)
    return jsonify({"ok": bool(axis) and controller.jog_hold(axis, data.get("id"))})

@app.route("/api/jog/release", methods=["POST"])
def api_jog_release():
    data = request.get_json(silent=True) or {}
    axis = _axis_arg(data)
    if axis:
        controller.jog_release(axis, data.get("id"))
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

@app.route("/api/manual/end", methods=["POST"])
def api_manual_end():
    ok, err = controller.end_manual_program()
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
    return jsonify({"ok": ok, "error": None if ok else "Automatic cycle is not available"}), (200 if ok else 409)

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