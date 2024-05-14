import fs from "fs";

export async function GET() {
  // load data
  const path = "../logs";
  const files = fs.readdirSync(path).sort();
  let raw_data = [];
  for (let i = 0; i < files.length; i += 1) {
    raw_data = raw_data.concat(
      JSON.parse(
        "[" + fs.readFileSync(`${path}/${files[i]}`, "utf-8").slice(0, -3) + "]"
      )
    );
  }
  // filter invalid GPS coords
  let data = [];
  for (let i = 0; i < raw_data.length; i += 1) {
    if (raw_data[i].GN === undefined || raw_data[i].GE === undefined) continue;
    else
      data.push({
        Tw: parseFloat(raw_data[i].Tw),
        Ta: parseFloat(raw_data[i].Ta),
        Ma: parseFloat(raw_data[i].Ma),
        Lo: parseInt(raw_data[i].Lo),
        GN: parseFloat(raw_data[i].GN),
        GE: parseFloat(raw_data[i].GE),
        time: raw_data[i].time,
      });
  }
  return Response.json({ data });
}
