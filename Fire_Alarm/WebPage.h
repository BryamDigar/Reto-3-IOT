const char* SSID = "OPPO A54";
const char* PASSWORD = "Arduino1234";
const int HTTP_PORT = 80;
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Sensor Data</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; text-align: center; background-color: #f0f0f0; margin: 0; padding: 0; }
    h1 { background-color: #4CAF50; color: white; padding: 20px; margin: 0; }
    .container { padding: 20px; }
    .data { font-size: 2em; margin: 10px 0; }
    .lcd { font-size: 1.5em; white-space: pre; background-color: #333; color: #fff; padding: 10px; border-radius: 5px; margin-bottom: 20px; }
    .alert { color: red; font-weight: bold; }
    .ok { color: green; font-weight: bold; }
    .warning { color: orange; font-weight: bold; }
    .button { background-color: #f44336; color: white; padding: 15px 25px; text-align: center; text-decoration: none; display: inline-block; font-size: 16px; margin: 4px 2px; cursor: pointer; border-radius: 4px; }
    .button:disabled { background-color: #cccccc; color: #666666; cursor: not-allowed; }
    .tab { overflow: hidden; border: 1px solid #ccc; background-color: #f1f1f1; }
    .tab button { background-color: inherit; float: left; border: none; outline: none; cursor: pointer; padding: 14px 16px; transition: 0.3s; }
    .tab button:hover { background-color: #ddd; }
    .tab button.active { background-color: #ccc; }
    .tabcontent { display: none; padding: 6px 12px; border: 1px solid #ccc; border-top: none; }
    #realtime { display: block; }
    .notifications { text-align: left; margin: 20px 0; padding: 10px; background-color: #f8f8f8; border-radius: 5px; }
    .notification-item { padding: 5px; margin: 5px 0; border-bottom: 1px solid #ddd; }
    table { width: 100%; border-collapse: collapse; margin: 20px 0; }
    th, td { border: 1px solid #ddd; padding: 8px; text-align: center; }
    th { background-color: #4CAF50; color: white; }
    tr:nth-child(even) { background-color: #f2f2f2; }
    .chart-container { height: 300px; width: 100%; }
  </style>
  <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/2.9.4/Chart.min.js"></script>
  <script>
    let tempChart, humiChart, coChart;
    let historyData = [];
    
    function fetchData() {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('temp').innerText = isNaN(data.temp) ? 'Null' : data.temp + ' ℃';
          document.getElementById('humi').innerText = isNaN(data.humi) ? 'Null' : data.humi + ' %';
          document.getElementById('co').innerText = isNaN(data.co) ? 'Null' : data.co;
          document.getElementById('lcd').innerText = data.lcd || 'Null';
          document.getElementById('status').innerText = data.status || 'Null';
          
          const disableButton = document.getElementById('disableAlarm');
          if (data.alarmActive) {
            document.getElementById('status').className = "alert";
            disableButton.disabled = false;
          } else if (data.status === "Temp alta" || data.status === "Temp baja" || data.status === "Humedad Baja") {
            document.getElementById('status').className = "warning";
            disableButton.disabled = true;
          } else {
            document.getElementById('status').className = "ok";
            disableButton.disabled = true;
          }
        });
    }
    
    function fetchHistory() {
      fetch('/history')
        .then(response => response.json())
        .then(data => {
          historyData = data;
          updateHistoryTable();
          updateCharts();
        });
    }
    
    function fetchNotifications() {
      fetch('/notifications')
        .then(response => response.json())
        .then(data => {
          const notifContainer = document.getElementById('notificationsList');
          notifContainer.innerHTML = '';
          
          data.forEach(notification => {
            const notifElem = document.createElement('div');
            notifElem.className = 'notification-item';
            notifElem.innerText = notification;
            notifContainer.appendChild(notifElem);
          });
        });
    }
    
    function disableAlarm() {
      fetch('/disableAlarm')
        .then(response => response.text())
        .then(data => {
          console.log(data);
          document.getElementById('disableAlarm').disabled = true;
          fetchNotifications();
        });
    }
    
    function updateHistoryTable() {
      const tableBody = document.getElementById('historyTableBody');
      tableBody.innerHTML = '';
      
      historyData.slice(0, 10).forEach(record => {
        const row = document.createElement('tr');
        
        const time = new Date(record.timestamp);
        const timeStr = time.getHours().toString().padStart(2, '0') + ':' + 
                        time.getMinutes().toString().padStart(2, '0') + ':' + 
                        time.getSeconds().toString().padStart(2, '0');
        
        const timeCell = document.createElement('td');
        timeCell.innerText = timeStr;
        row.appendChild(timeCell);
        
        const tempCell = document.createElement('td');
        tempCell.innerText = isNaN(record.temp) ? 'Null' : record.temp.toFixed(1) + ' ℃';
        row.appendChild(tempCell);
        
        const humiCell = document.createElement('td');
        humiCell.innerText = isNaN(record.humi) ? 'Null' : record.humi.toFixed(1) + ' %';
        row.appendChild(humiCell);
        
        const coCell = document.createElement('td');
        coCell.innerText = isNaN(record.co) ? 'Null' : record.co;
        row.appendChild(coCell);
        
        const statusCell = document.createElement('td');
        statusCell.innerText = record.status || 'Null';
        if (record.status === "ALERTA INCENDIO!") {
          statusCell.className = "alert";
        } else if (record.status !== "Monitoreo OK") {
          statusCell.className = "warning";
        }
        row.appendChild(statusCell);
        
        tableBody.appendChild(row);
      });
    }
    
    function updateCharts() {
      const reversedData = [...historyData].reverse();
      
      const labels = reversedData.map(record => {
        const time = new Date(record.timestamp);
        return time.getHours().toString().padStart(2, '0') + ':' + 
               time.getMinutes().toString().padStart(2, '0');
      });
      
      const tempData = reversedData.map(record => record.temp);
      const humiData = reversedData.map(record => record.humi);
      const coData = reversedData.map(record => record.co);
      
      tempChart.data.labels = labels;
      tempChart.data.datasets[0].data = tempData;
      tempChart.update();
      
      humiChart.data.labels = labels;
      humiChart.data.datasets[0].data = humiData;
      humiChart.update();
      
      coChart.data.labels = labels;
      coChart.data.datasets[0].data = coData;
      coChart.update();
    }
    
    function initCharts() {
      const tempCtx = document.getElementById('tempChart').getContext('2d');
      tempChart = new Chart(tempCtx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [{
            label: 'Temperatura (°C)',
            data: [],
            backgroundColor: 'rgba(255, 99, 132, 0.2)',
            borderColor: 'rgba(255, 99, 132, 1)',
            borderWidth: 2,
            tension: 0.3
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            y: {
              beginAtZero: false
            }
          }
        }
      });
      
      const humiCtx = document.getElementById('humiChart').getContext('2d');
      humiChart = new Chart(humiCtx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [{
            label: 'Humedad (%)',
            data: [],
            backgroundColor: 'rgba(54, 162, 235, 0.2)',
            borderColor: 'rgba(54, 162, 235, 1)',
            borderWidth: 2,
            tension: 0.3
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            y: {
              beginAtZero: false
            }
          }
        }
      });
      
      const coCtx = document.getElementById('coChart').getContext('2d');
      coChart = new Chart(coCtx, {
        type: 'line',
        data: {
          labels: [],
          datasets: [{
            label: 'Nivel CO',
            data: [],
            backgroundColor: 'rgba(255, 206, 86, 0.2)',
            borderColor: 'rgba(255, 206, 86, 1)',
            borderWidth: 2,
            tension: 0.3
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          scales: {
            y: {
              beginAtZero: true
            }
          }
        }
      });
    }
    
    function openTab(evt, tabName) {
      const tabcontent = document.getElementsByClassName("tabcontent");
      for (let i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
      }
      
      const tablinks = document.getElementsByClassName("tablinks");
      for (let i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
      }
      
      document.getElementById(tabName).style.display = "block";
      evt.currentTarget.className += " active";
      
      if (tabName === 'history') {
        fetchHistory();
      } else if (tabName === 'notifications') {
        fetchNotifications();
      }
    }
    
    window.onload = function() {
      document.getElementById("defaultTab").click();
      initCharts();
      fetchData();
      fetchHistory();
      fetchNotifications();
      setInterval(fetchData, 5000);
      setInterval(fetchHistory, 10000);
      setInterval(fetchNotifications, 5000);
    };
  </script>
</head>
<body>
  <h1>ESP32 Sensor Data</h1>
  
  <div class="tab">
    <button class="tablinks active" id="defaultTab" onclick="openTab(event, 'realtime')">Tiempo Real</button>
    <button class="tablinks" onclick="openTab(event, 'history')">Histórico</button>
    <button class="tablinks" onclick="openTab(event, 'charts')">Gráficos</button>
    <button class="tablinks" onclick="openTab(event, 'notifications')">Notificaciones</button>
  </div>
  
  <div id="realtime" class="tabcontent">
    <div class="container">
      <div id="status" class="ok">Monitoreo OK</div>
      <div class="data">Temperatura: <span id="temp"></span></div>
      <div class="data">Humedad: <span id="humi"></span></div>
      <div class="data">Nivel CO: <span id="co"></span></div>
      <div class="lcd" id="lcd"></div>
      <button id="disableAlarm" class="button" onclick="disableAlarm()" disabled>Desactivar Alarma</button>
    </div>
  </div>
  
  <div id="history" class="tabcontent">
    <div class="container">
      <h2>Histórico de Datos</h2>
      <table>
        <thead>
          <tr>
            <th>Hora</th>
            <th>Temperatura</th>
            <th>Humedad</th>
            <th>Nivel CO</th>
            <th>Estado</th>
          </tr>
        </thead>
        <tbody id="historyTableBody">
          <!-- Datos históricos se cargarán aquí -->
        </tbody>
      </table>
    </div>
  </div>
  
  <div id="charts" class="tabcontent">
    <div class="container">
      <h2>Gráficos</h2>
      <div class="chart-container">
        <canvas id="tempChart"></canvas>
      </div>
      <div class="chart-container">
        <canvas id="humiChart"></canvas>
      </div>
      <div class="chart-container">
        <canvas id="coChart"></canvas>
      </div>
    </div>
  </div>
  
  <div id="notifications" class="tabcontent">
    <div class="container">
      <h2>Notificaciones</h2>
      <div class="notifications" id="notificationsList">
        <!-- Notificaciones se cargarán aquí -->
      </div>
    </div>
  </div>
  
</body>
</html>
)=====";