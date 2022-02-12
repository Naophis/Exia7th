var m = 0.015;

var accl = 1;

var r = 12 / 2;

var resist = 2;

var gear = 37 / 8;

var km = 0.00020;

var result = (m * accl * r) / resist / gear / km;

console.log(result);

var v = 1000;
v /= 1000;

var result2 = result * v;
console.log(result2);

console.log((m * r * accl) / 2 / gear);
