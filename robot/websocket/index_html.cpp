#include "websocket.h"
#include <string.h>

const char html[] = R"rawliteral(
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
    <div class="card">
      <h2>Output - GPIO 4</h2>
      <p class="state">state: <span id="state">%STATE%</span></p>
      <p><button id="button" class="button">Toggle</button></p>
    </div>
  </div>
  <div class="content">
    <table>
    <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('forward');" onmouseup="toggleCheckbox('stop');" ontouchstart="forward(event);"  ontouchcancel="stop(event);" ontouchend="stop(event);">Forward</button></td></tr>
    <tr><td align="center"><button class="button" onmousedown="toggleCheckbox('left');" onmouseup="toggleCheckbox('stop');" ontouchstart="left(event);"  ontouchcancel="stop(event);" ontouchend="stop(event);">Left</button></td><td align="center">
    <button class="button" onmousedown="toggleCheckbox('stop');" ontouchstart="stop(event);">Stop</button></td><td align="center">
    <button class="button" onmousedown="toggleCheckbox('right');" ontouchstart="right(event);" onmouseup="toggleCheckbox('stop');" ontouchcancel="stop(event);" ontouchend="stop(event);">Right</button></td></tr>
    <tr><td colspan="3" align="center"><button class="button" onmousedown="toggleCheckbox('backward');" ontouchstart="backward(event);" onmouseup="toggleCheckbox('stop');" ontouchcancel="stop(event);" ontouchend="stop(event);">Backward</button></td></tr>
    </table>
  </div>

  <script>
  class WebSocketClient {
      constructor() {
        this.gateway = 'ws://${window.location.hostname}/ws';
        this.websocket = null;
      }

      init() {
        console.log('Trying to open a WebSocket connection...');
        this.websocket = new WebSocket(this.gateway);
        this.websocket.onopen = this.onOpen.bind(this);
        this.websocket.onclose = this.onClose.bind(this);
        this.websocket.onmessage = this.onMessage.bind(this);
      }

      onOpen(event) {
        console.log('Connection opened');
      }

      onClose(event) {
        console.log('Connection closed');
        setTimeout(this.init.bind(this), 2000);
      }

      onMessage(event) {
        const state = event.data === "1" ? "ON" : "OFF";
        document.getElementById('state').innerHTML = state;
      }
    }

    document.addEventListener('DOMContentLoaded', function() {
      const socketClient = new WebSocketClient();
      socketClient.init();

      document.getElementById('button').addEventListener('click', function() {
        socketClient.websocket.send('toggle');
      });
    });


   function forward(event) {
     event.preventDefault()
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=forward", true);
     xhr.send();
   }

   function stop(event) {
     event.preventDefault()
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=stop", true);
     xhr.send();
   }

   function left(event) {
     event.preventDefault()
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=left", true);
     xhr.send();
   }

   function right(event) {
     event.preventDefault()
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=right", true);
     xhr.send();     
   }

   function backward(event) {
     event.preventDefault()
     var xhr = new XMLHttpRequest();
     xhr.open("GET", "/action?go=backward", true);
     xhr.send();
   }
</script>
</body>
</html>
)rawliteral";

PGM_P getIndexHtml() {
  // static char combinedHtml[sizeof(html) + sizeof(js)];
  // strcpy_P(combinedHtml, html);
  // strcat_P(combinedHtml, js);
  return html;
}