#include "websocket.h"
#include <string.h>
#include <pgmspace.h>

const char html_str[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
  html {
    font-family: Arial, Helvetica, sans-serif;
    text-align: center;
  }
  h1 {
    font-size: 1.8rem;
    color: white;
  }
  h2{
    font-size: 1.5rem;
    font-weight: bold;
    color: #143642;
  }
  .topnav {
    overflow: hidden;
    background-color: #143642;
  }
  body {
    margin: 0;
  }
  .content {
    padding: 30px;
    max-width: 600px;
    margin: 0 auto;
  }
  .card {
    background-color: #F8F7F9;;
    box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
    padding-top:10px;
    padding-bottom:20px;
  }
  .button {
    padding: 15px 50px;
    font-size: 24px;
    text-align: center;
    outline: none;
    color: #fff;
    background-color: #0f8b8d;
    border: none;
    border-radius: 5px;
    -webkit-touch-callout: none;
    -webkit-user-select: none;
    -khtml-user-select: none;
    -moz-user-select: none;
    -ms-user-select: none;
    user-select: none;
    -webkit-tap-highlight-color: rgba(0,0,0,0);
   }
   /*.button:hover {background-color: #0f8b8d}*/
   .button:active {
     background-color: #0f8b8d;
     box-shadow: 2 2px #CDCDCD;
     transform: translateY(2px);
   }
   .state {
     font-size: 1.5rem;
     color:#8c8c8c;
     font-weight: bold;
   }
  </style>
</head>
<body>
  <div class="topnav">
    <h1>ESP WebSocket Server</h1>
  </div>
  <div class="content">
      <img id="image"/>
  </div>
  <div class="content">
    <div class="card">
      <table align="center">
      <tr><td colspan="3" align="center"><button class="button" id="forward">Forward</button></td></tr>
      <tr><td align="center"><button class="button" id="left">Left</button></td><td align="center">
      <button class="button" id="stop">Stop</button></td><td align="center">
      <button class="button" id="right">Right</button></td></tr>
      <tr><td colspan="3" align="center"><button class="button" id="backward">Backward</button></td></tr>
      </table>
    </div>
  </div>
  
  <script>
  // var gateway = 'ws://${window.location.hostname}/ws';
  const gateway = "ws://192.168.10.102/ws";
  

  class WebSocketClient {
      constructor() {
        this.websocket = null;
      }

      init() {
        console.log('Trying to open a WebSocket connection...');  
        this.websocket = new WebSocket(gateway);      
        this.websocket.onopen = this.onOpen.bind(this);
        this.websocket.onclose = this.onClose.bind(this);
        this.websocket.onmessage = this.onMessage.bind(this);
      }

      onOpen(event) {
        console.log('Connection opened');
      }

      onClose(event) {
        console.log('Connection closed');
        setTimeout(this.init.bind(this), 500);
      }

      onMessage(event) {
        const imageBlob = new Blob([event.data], { type: 'image/jpeg' });
        document.getElementById('image').src = URL.createObjectURL(imageBlob);
        // const state = event.data === "1" ? "ON" : "OFF";
        // document.getElementById('state').innerHTML = state;
      }
    }
    
    document.addEventListener("DOMContentLoaded", function() {
      const socketClient = new WebSocketClient();
      socketClient.init();

      // document.getElementById("button").addEventListener("click", function(e) {
      //   socketClient.websocket.send('toggle');
      //   test('hello');
      // });

      document.getElementById("stop").addEventListener("mousedown", function(e) {
        socketClient.websocket.send('stop');
      });

      for (let evt_name of ["forward","backward","left","right"]) {
        document.getElementById(evt_name).addEventListener("mousedown", function(e) {
          socketClient.websocket.send(evt_name);
        });

        document.getElementById(evt_name).addEventListener("mouseup", function(e) {
         socketClient.websocket.send('stop');
        });
      }
    });
</script>
</body>
</html>
)rawliteral";

char *getIndexHtml()
{
  size_t len = strlen_P(html_str) + 1;
  char *html = new char[len];

  strcpy_P(html, html_str);
  // static char combinedHtml[sizeof(html) + sizeof(js)];
  // strcpy_P(combinedHtml, html);
  // strcat_P(combinedHtml, js);
  return html;
}