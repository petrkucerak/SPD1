"use client";
import Graph from "@/components/graph";
import { base_url } from "@/components/utils";
import { useSearchParams } from "next/navigation";
import { useEffect, useState } from "react";

const getData = async () => {
  const res = await fetch(`${base_url}/api/data?GN=${GN}&GE=${GE}`);
  const json = await res.json();
  return { data: json };
};

export default function Page() {
  const [stationData, setStationData] = useState(null);
  const [isLoading, setLoading] = useState(true);

  const searchParams = useSearchParams();
  const GE = searchParams.get("GE");
  const GN = searchParams.get("GN");

  useEffect(() => {
    fetch(`${base_url}/api/data?GN=${GN}&GE=${GE}`)
      .then((res) => res.json())
      .then((data) => {
        setStationData(data.data);
        setLoading(false);
      });
  }, []);

  if (isLoading) return <p>Loading...</p>;
  if (!stationData) return <p>No station data</p>;

  return (
    <div className="w-[90vw] max-w-[800px]">
      <h1 className="text-2xl font-bold">Stanice</h1>
      <span className="font-semibold">
        {GN} {GE}
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
