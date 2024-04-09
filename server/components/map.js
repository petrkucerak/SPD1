"use client";
import { MapContainer, TileLayer } from "react-leaflet";

export default function Map() {
  const position = [51.505, -0.09];

  if (typeof window !== undefined) {
    return (
      <MapContainer
        zoom={16}
        center={position}
        scrollWheelZoom={true}
        className="w-screen h-[70vh] diseable-map-selection z-0"
      >
        {" "}
        <TileLayer url="https://{s}.tile.openstreetmap.fr/hot/{z}/{x}/{y}.png" />
      </MapContainer>
    );
  }
}
