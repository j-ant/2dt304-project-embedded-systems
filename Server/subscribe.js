const mqtt = require("mqtt");
const WebSocket = require('ws');
const WeatherData = require('./app');

const wss = new WebSocket.Server({ port: 8080 });

const mqttHost = "172.20.10.4";
const protocol = "mqtt"
const port = "1883";

let connections = [];

wss.on('connection', function connection(ws) {
  connections.push(ws);

  ws.on('close', function() {
    connections = connections.filter(conn => conn !== ws);
  });
});

function connectToBroker() {
    const clientId = "client" + Math.random().toString(36).substring(7);

    const hostURL = `${protocol}://${mqttHost}:${port}`;

    const options = {
        keepalive: 60,
        clientId: clientId,
        protocolId: "MQTT",
        protocolVersion: 4,
        clean: true,
        reconnectPeriod: 1000,
        connectTimeout: 30* 1000,
    };

    mqttClient = mqtt.connect(hostURL, options);

    mqttClient.on("error", (err) => {
        console.log("Error: ", err);
        mqttClient.end();
    });

    mqttClient.on("reconnect", () => {
        console.log("Reconnecting...");
    });

    mqttClient.on("connect", () => {
        console.log("Client connected:" + clientId);
    })

    mqttClient.on("message", (topic, message, packet) => {
    
        if (topic === 'sensor/data') {
            try {
                // Parse the incoming message as JSON
                console.log(message.toString())
                const sensorData = JSON.parse(message.toString());
    
                // Construct the message to send over WebSocket
                const dataToBroadcast = JSON.stringify({
                    timestamp: new Date(),
                    temperature: sensorData.Temperature,
                    humidity: sensorData.Humidity,
                    pressure: sensorData.Pressure,
                    gas: sensorData.Gas
                });
    
                // send message to all connected WebSocket clients
                connections.forEach(conn => conn.send(dataToBroadcast));
    
                // Prepare the data to save to the database
                let dataToSave = {
                    timestamp: new Date(),
                    temperature: sensorData.Temperature,
                    humidity: sensorData.Humidity,
                    pressure: sensorData.Pressure,
                    gas: sensorData.Gas
                };
    
                // Create and save the new document
                const newData = new WeatherData(dataToSave);
                newData.save().then(doc => console.log("Document saved:", doc))
                              .catch(err => console.error("Error saving document:", err));
    
            } catch (err) {
                console.error("Error parsing JSON message:", err);
            }
        } else {
            console.log('Unknown topic or not handling this topic here', topic);
        }
    });

}

function subscribe(topic) {
    console.log(`"Subscribing to Topic: ${topic}`);

    mqttClient.subscribe(topic, { qos: 0});
}

connectToBroker();
subscribe("sensor/data");

module.exports = {
    connectToBroker,
    subscribe,
};