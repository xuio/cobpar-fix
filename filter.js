const alpha = 0.01;
let buffer = 0;
let out = [];

let rgb_val = [255, 255, 255];

let reached = false;

for(let i=0; i < 600; i++){
	const avg_val = (rgb_val[0] + rgb_val[1] + rgb_val[2]) / 3;
	buffer = (alpha * avg_val) + (1.0 - alpha) * buffer;
	out.push(parseInt(buffer));
	if(buffer > 200 && !reached){
		console.log(`reached after ${i/26} seconds, ${i} frames`);
		reached = !reached;
	}
}

console.log(JSON.stringify(out));
