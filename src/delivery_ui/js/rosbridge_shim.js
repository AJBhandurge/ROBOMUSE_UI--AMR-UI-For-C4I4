(function(){
"use strict";

/* Fix rosbridge's broken int8 array serialization:
   [-1,-1,,1,-1] → double commas (missing values)
   We clean these before parsing. */
function fixBrokenJson(s) {
  // Replace double commas like ",,", ",," with ",null,"
  return s.replace(/,\s*,/g, ',null,')
          // Also fix leading comma in array [,-1] 
          .replace(/\[,/g, '[null,')
          // Fix trailing comma before ] 
          .replace(/,\s*\]/g, ']');
}

/* Safe JSON.parse — never throws, returns null on failure */
var _origParse = JSON.parse;
JSON.parse = function(text, rev) {
  if (typeof text !== "string") return _origParse(text, rev);
  try { return _origParse(text, rev); }
  catch(e) {
    // Try fixing broken int8 arrays first
    try { return _origParse(fixBrokenJson(text), rev); }
    catch(e2) { return null; }
  }
};

/* WebSocket patch — reassembles fragmented frames */
var _N = window.WebSocket;

function makeR(fn) {
  var buf="",dep=0,inS=false,esc=false,tmr=null;
  function reset(){buf="";dep=0;inS=false;esc=false;if(tmr){clearTimeout(tmr);tmr=null;}}
  function arm(){if(tmr)clearTimeout(tmr);tmr=setTimeout(function(){reset();},2000);}
  function scan(c){
    for(var i=0;i<c.length;i++){
      var ch=c[i];
      if(esc){esc=false;continue;}
      if(inS){if(ch==="\\")esc=true;else if(ch==='"')inS=false;continue;}
      if(ch==='"'){inS=true;continue;}
      if(ch==='{'||ch==='['){dep++;continue;}
      if(ch==='}'||ch===']'){if(--dep===0)return true;}
    }
    return false;
  }
  return function(e){
    var d=e.data;
    if(d instanceof ArrayBuffer||d instanceof Blob)return;
    if(typeof d==="string"){
      var fc=d.charCodeAt(0);
      if(fc!==123&&fc!==91)return;
    }
    /* Fast path: single complete valid frame */
    if(buf.length===0){
      var p=JSON.parse(d);
      if(p!==null){fn(e);return;}
      /* Invalid but starts with { — could be fragment, start buffering */
    }
    buf+=d; arm();
    if(!scan(d))return;
    /* Complete frame assembled */
    var full=buf; reset();
    var p2=JSON.parse(full);
    if(p2===null)return;
    fn(new MessageEvent("message",{data:full,origin:e.origin}));
  };
}

function PWS(url,pro){
  var ws=pro?new _N(url,pro):new _N(url);
  var self=this,_w=[],_om=null;
  ws.binaryType="arraybuffer";
  self.addEventListener=function(t,f,o){
    if(t==="message"){var w=makeR(f);_w.push({o:f,w:w});ws.addEventListener("message",w,o);}
    else ws.addEventListener(t,f,o);
  };
  self.removeEventListener=function(t,f,o){
    if(t==="message"){for(var i=0;i<_w.length;i++){if(_w[i].o===f){ws.removeEventListener("message",_w[i].w,o);_w.splice(i,1);return;}}}
    ws.removeEventListener(t,f,o);
  };
  Object.defineProperty(self,"onmessage",{
    get:function(){return _om;},
    set:function(f){_om=f;ws.onmessage=f?makeR(f):null;},
    enumerable:true,configurable:true
  });
  ["onopen","onclose","onerror"].forEach(function(ev){
    Object.defineProperty(self,ev,{get:function(){return ws[ev];},set:function(f){ws[ev]=f;},enumerable:true,configurable:true});
  });
  ["readyState","bufferedAmount","extensions","protocol","url"].forEach(function(p){
    Object.defineProperty(self,p,{get:function(){return ws[p];},enumerable:true,configurable:true});
  });
  Object.defineProperty(self,"binaryType",{
    get:function(){return ws.binaryType;},set:function(v){ws.binaryType=v;},
    enumerable:true,configurable:true
  });
  self.send=function(d){ws.send(d);};
  self.close=function(c,r){ws.close(c,r);};
  self.dispatchEvent=function(e){return ws.dispatchEvent(e);};
  self.CONNECTING=0;self.OPEN=1;self.CLOSING=2;self.CLOSED=3;
}
PWS.CONNECTING=0;PWS.OPEN=1;PWS.CLOSING=2;PWS.CLOSED=3;
window.WebSocket=PWS;
console.log("[rosbridge_shim] Patched — fixBrokenJson + reassembly active");
})();