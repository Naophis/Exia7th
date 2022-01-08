var fs = require("fs");
const yaml = require("js-yaml");
let SerialPort = require("serialport");
const Readline = require("@serialport/parser-readline");

let comport;
let port;

let parser;

function resolveAfter2Seconds(str) {
  return new Promise((resolve) => {
    setTimeout(() => {
      resolve(str);
      console.log(str); // 10
    }, 500);
  });
}

async function f1(str) {
  var x = await resolveAfter2Seconds(str);
}
let ready = function () {
  port = new SerialPort(
    comport,
    {
      baudRate: 2000000,
      // baudRate: 115200,
    },
    (e) => {
      if (e) {
        console.log("comport access dinied");
      } else {
        console.log("connect");
      }
    }
  );
  parser = port.pipe(
    new Readline({
      delimiter: "\r\n",
    })
  );
  function getNowYMD() {
    var dt = new Date();
    var y = dt.getFullYear();
    var m = ("00" + (dt.getMonth() + 1)).slice(-2);
    var d = ("00" + dt.getDate()).slice(-2);
    var h = ("00" + dt.getHours()).slice(-2);
    var M = ("00" + dt.getMinutes()).slice(-2);
    var s = ("00" + dt.getSeconds()).slice(-2);
    return `${y}${m}${d}_${h}${M}_${s}.csv`;
  }
  let dump_to_csv = false;
  let file_name = "";

  parser.on("data", function (data) {
    console.log(data);

    if (data.match(/^end___/)) {
      dump_to_csv = false;
      fs.copyFile(
        `${__dirname}/logs/${file_name}`,
        `${__dirname}/logs/latest.csv`,
        (err) => {
          if (err) {
            console.log(err.stack);
          } else {
            console.log("Copy Done.");
          }
        }
      );
    }
    if (dump_to_csv) {
      fs.appendFileSync(`${__dirname}/logs/${file_name}`, `${data}\n`, {
        flag: "a",
      });
    }

    if (data.match(/^start___/)) {
      dump_to_csv = true;
      file_name = getNowYMD();
      console.log(file_name);
    }
  });
};

SerialPort.list().then(
  (ports) => {
    for (let i in ports) {
      const p = ports[i];
      console.log(p.path, p.serialNumber);
      if (
        p.path.match(/usbserial/) ||
        p.path.match(/COM/) ||
        p.path.match(/ttyUSB/)
      ) {
        if (p.serialNumber) {
          comport = p.path;
          console.log(`select: ${comport}`);
          ready();
          break;
        }
      }
    }
  },
  (err) => console.error(err)
);
