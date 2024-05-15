import fs from "fs";

function isInArea(accuracy, GN, GE, p_GN, p_GE) {
  if (p_GN > GN + accuracy) return false;
  if (p_GN < GN - accuracy) return false;
  if (p_GE > GE + accuracy) return false;
  if (p_GE < GE - accuracy) return false;
  return true;
}

export async function GET(request) {
  // load data
  const { searchParams } = new URL(request.url);
  const GN = parseFloat(searchParams.get("GN"));
  const GE = parseFloat(searchParams.get("GE"));
  const accuracy = parseFloat(searchParams.get("accuracy"));

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
    else {
      // filter data by params
      if (GN !== null && GE !== null) {
        // if is not same as GN and GE skip it
        if (
          // parseFloat(raw_data[i].GN) !== GN ||
          // parseFloat(raw_data[i].GE) !== GE
          !isInArea(accuracy, GN, GE, raw_data[i].GN, raw_data[i].GE)
        )
          continue;
      }
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
  }
  return Response.json({ data });
}
