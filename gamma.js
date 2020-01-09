let gamma   = 2.8; // Correction factor
const max_in  = 255; // Top end of INPUT range
const max_out = 65534; // Top end of OUTPUT range

let out = [];

for(let i=0; i <= max_in; i++) {
  out.push(parseInt((i / max_in)**(gamma) * max_out + 0.5) + 1);
}
console.log(JSON.stringify(out));
