"use client";
import Graph from "@/components/graph";
import { base_url } from "@/components/utils";
import { useSearchParams } from "next/navigation";
import { useEffect, useState } from "react";

export default function Page() {
  const [stationData, setStationData] = useState(null);
  const [isLoading, setLoading] = useState(true);

  const searchParams = useSearchParams();
  const GE = searchParams.get("GE");
  const GN = searchParams.get("GN");
  const accuracy = searchParams.get("accuracy");

  useEffect(() => {
    fetch(`${base_url}/api/data?GN=${GN}&GE=${GE}&accuracy=${accuracy}`)
      .then((res) => res.json())
      .then((data) => {
        setStationData(data.data);
        setLoading(false);
      });
  }, []);

  if (isLoading) return <p>Načítání...</p>;
  if (!stationData) return <p>Data pro tuto stanici neexistují</p>;

  return (
    <div className="w-[90vw] max-w-[800px]">
      <h1 className="text-2xl font-bold">Stanice</h1>
      <span className="font-semibold">
        ({GN}, {GE} ±{accuracy})
      </span>
      <Graph
        data={stationData}
        type="Lo"
        title={"Světelnost"}
        label={"Světelnost"}
      />
      <Graph
        data={stationData}
        type={"Ta Tw"}
        title={"Teplota vzduchu a vody"}
        label={"Teplota"}
      />
      <Graph
        data={stationData}
        type="Ma"
        title={"Vlhkost vzduchu"}
        label={"Vlkohst"}
      />
    </div>
  );
}
