const mongoose = require("mongoose");

const dbUrl = "mongodb+srv://Felix:gcIM5g81DUODlwad@cluster0.uqb5kxy.mongodb.net/weatherstation?retryWrites=true&w=majority&appName=Cluster0";

const connectionParams = {
    useNewUrlParser: true,
    useUnifiedTopology: true,
};

mongoose
.connect(dbUrl, connectionParams)
.then(() => {
    console.info("Connected to DB");
})
.catch((e) => {
    console.log("Error: ", e);
});

const skolaStation = new mongoose.Schema({
    timestamp: { type: Date, required: true },
    temperature: { type: Number, required: false, default: null },
    humidity: { type: Number, required: false, default: null },
    pressure: { type: Number, required: false, default: null },
    gas: { type: Number, required: false, default: null } 
  });

const WeatherData = mongoose.model('WeatherData', skolaStation);

module.exports = WeatherData;
