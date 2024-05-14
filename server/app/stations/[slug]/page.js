import Graph from "@/components/graph";
import fs from "fs";

export function loadData() {
  const dir = fs.readdirSync("../logs").sort();
  let data = [];
  for (let i = 0; i < dir.length; i += 1) {
    data = data.concat(
      JSON.parse(
        "[" + fs.readFileSync(`../logs/${dir[i]}`, "utf-8").slice(0, -3) + "]"
      )
    );
  }
  return data;
}

export async function generateStaticParams() {
  const data = loadData();
  let stations = [];
  let stations_text = [];
  for (let i = 0; i < data.length; ++i) {
    if (!stations_text.includes(`${data[i].GN},${data[i].GE}`)) {
      stations.push({ lat: data[i].GN, lan: data[i].GE });
      stations_text.push(`${data[i].GN},${data[i].GE}`);
    }
  }
  return stations.map((station) => ({
    slug: `${station.lat}-${station.lan}`,
  }));
}

export default function Page({ params }) {
  const data = loadData();
  const coord = params.slug.split("-");
  const station_data = data.filter(
    (record) => record.GN == coord[0] && record.GE == coord[1]
  );
  return (
    <div className="w-[90vw] max-w-[800px]">
      <h1 className="text-2xl font-bold">Stanice</h1>
      <span className="font-semibold">
        {coord[0]} {coord[1]}
      </span>
      <Graph
        data={station_data}
        type="light"
        title={"Světelnost"}
        label={"Světelnost"}
      />
      <Graph
        data={station_data}
        type="temperature"
        title={"Teplota"}
        label={"Teplota"}
      />
    </div>
  );
}
