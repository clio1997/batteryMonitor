var fs = require('fs');

var debugLevel = 0;

if (process.argv.length > 2) {
  logFileName = process.argv[2];
  if (process.argv.length > 3) {
    if (process.argv[3] == "-d") {
      debugLevel = 1;
    }
  }
} else {
  console.log("Usage: computeAh.js logfilename");
  process.exit();
}

var shuntOhms = .0003787878;
var asecs = 0;
var seconds = 0;
var linesAccepted = 0;
var linesRejected = 0;


fs.readFile(logFileName, (err,data) => {
  var content = data.toString().split("\n");

  var voltageFactor = 1.0;
  var ampSum = 0.0;
  
  var firstTimestamp = -1;
  var lastTimestamp = 0;

  for (var i=0; i<content.length;i++) {
    var curLine = content[i].split(" ");
    if (curLine.length >= 4) {
      if (curLine[2] == "mV") {
        voltageFactor = 0.001;
      } else {
        voltageFactor = 1.0;
      }

      var volts = Math.abs(parseFloat(curLine[1]) * voltageFactor);
      var amps = volts / shuntOhms;

      if ((amps > 40) && (amps < 80)) {
        ampSum += amps;
        linesAccepted++;
        if (firstTimestamp < 0) {
          firstTimestamp = curLine[0];
        }
        
        lastTimestamp = curLine[0];
      } else {
        linesRejected++;
        if (debugLevel > 0)
          console.log("rejected " + amps + " amps at " + curLine[0] + " seconds");
      }
    }
  }

  if (linesAccepted > 0) {
    var meanAmps = ampSum / linesAccepted;
    var dischargeSeconds = lastTimestamp - firstTimestamp;
    console.log("Discharge time [seconds]: " +  dischargeSeconds + ", Ah: " + meanAmps * dischargeSeconds / 3600 + ", Lines accepted: " + linesAccepted + ", rejected: " + linesRejected);
  } else {
    console.log("No valid samples in file!");
  }

} );
