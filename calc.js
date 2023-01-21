var v = 7.500;
var r = 0.013 / 2;
var dt = 0.001;
var gear_ratio = 37.0 / 9;
// var gear_ratio = 9.0 / 37;

// console.log(w)
// console.log(theta * 180 / 3.14 * gear_ratio)
// console.log(theta * 180 / 3.14)


console.log(`d_v[m/s] = 2πr * pulse/resolution / dt / gear_ratio`)
console.log(`pulse = 1`)
function calc_v(n, gear_ratio) {
  var v = 1.0 / Math.pow(2, n) * Math.PI / gear_ratio * 2 * r / dt * 1000;
  console.log(`resolution:${n}, gear_ratio: ${gear_ratio.toFixed(2)}, d_v[mm/s]: ${v.toFixed(6)}`);
}

calc_v(12, 1.0);
calc_v(12, gear_ratio);
calc_v(13, 1.0);
calc_v(13, gear_ratio);
calc_v(14, 1.0);
calc_v(14, gear_ratio);


function calc_req_enc_spec(v, r, gear_ratio) {
  var w = v / r;
  var theta = w * dt;
  var ratio = theta * 180 / Math.PI * gear_ratio;
  var rpm = 30 / Math.PI * w;
  console.log(`req_deg/msec: ${ratio.toFixed(2)}[deg/msec], rpm: ${rpm.toFixed(2)}`)

}
calc_req_enc_spec(v, r, gear_ratio);

var right = 400;
var a = 417.79380553801263;
// var b = 29.122723581924973;
var b = 24.122723581924973;

function calc_dist(data, a, b) {
  return a / Math.log(a) - b;
}
console.log(calc_dist(right, a, b));