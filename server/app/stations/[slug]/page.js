import Graph from "@/components/graph";
import fs from "fs";

export async function generateStaticParams() {
  const data = fs.readFileSync("public/tmp/data_logs.json");
  let stations = [];
  let stations_text = [];
  for (let i = 0; i < data.length; ++i) {
    if (!stations_text.includes(`${data[i].gps_lat},${data[i].gps_lan}`)) {
      stations.push({ lat: data[i].gps_lat, lan: data[i].gps_lan });
      stations_text.push(`${data[i].gps_lat},${data[i].gps_lan}`);
    }
  }
  return stations.map((station) => ({
    slug: `${station.lat}-${station.lan}`,
  }));
}

export default function Page({ params }) {
  const data = JSON.parse(fs.readFileSync("public/tmp/data_logs.json"));
  const coord = params.slug.split("-");
  const station_data = data.filter(
    (record) => record.gps_lat == coord[0] && record.gps_lan == coord[1]
  );
  return (
    <div className="w-[90vw] max-w-[800px]">
      <h1 className="text-2xl font-bold">Stanice</h1>
      <span className="font-semibold">
        {coord[0]} {coord[1]}
      </span>
      <Graph data={station_data} type="light" title={"Světelnost"} label={"Světelnost"} />
      <Graph data={station_data} type="temperature" title={"Teplota"} label={"Teplota"} />
    </div>
  );
}
