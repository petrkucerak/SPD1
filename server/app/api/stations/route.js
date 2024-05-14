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
    else if (
      data.find((station) => station.GN === parseFloat(raw_data[i].GN)) ===
        undefined &&
      data.find((station) => station.GE === parseFloat(raw_data[i].GE)) ===
        undefined
    )
      data.push({
        GN: parseFloat(raw_data[i].GN),
        GE: parseFloat(raw_data[i].GE),
      });
  }
  return Response.json({ data });
}
