"use client";

import dynamic from "next/dynamic";
export default function Stations() {
  const MapWithNoSSR = dynamic(() => import("@/components/map"), {
    ssr: false,
  });
  return (
    <div className="w-full">
      <h1>Stanice</h1>
      <MapWithNoSSR />
    </div>
  );
}
