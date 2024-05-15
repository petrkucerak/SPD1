import fs from "fs";
import { isInArea } from "../data/route";

function isInSameArea(data, accuracy, GN, GE) {
  if (data.length === 0) return false;
  for (let i = 0; i < data.length; i += 1) {
    if (isInArea(accuracy, data[i].GN, data[i].GE, GN, GE)) return true;
  }
  return false;
}

export async function GET(request) {
  const { searchParams } = new URL(request.url);
  const accuracy = parseFloat(searchParams.get("accuracy"));
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
    ) {
      // handle accuracy
      if (
        !isInSameArea(
          data,
          accuracy,
          parseFloat(raw_data[i].GN),
          parseFloat(raw_data[i].GE)
        )
      ) {
        data.push({
          GN: parseFloat(raw_data[i].GN),
          GE: parseFloat(raw_data[i].GE),
        });
      }
    }
  }
  return Response.json({ data });
}
