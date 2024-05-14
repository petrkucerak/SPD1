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
    slug: `${station.lan}-${station.lat}`,
  }));
}

export default function Page({ params }) {
  return (
    <div className="w-[90vw] max-w-[800px]">
      <h1>Hello</h1>
      <Graph />
    </div>
  );
}
