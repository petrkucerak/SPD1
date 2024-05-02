import { MapConsumer, MapContainer, TileLayer } from "react-leaflet";
import { MAP_API_KEY } from "./utils";
import "leaflet-defaulticon-compatibility";
import "leaflet-defaulticon-compatibility/dist/leaflet-defaulticon-compatibility.css";
import "leaflet/dist/leaflet.css";

export default function Map() {
  return (
    <MapContainer
      center={[50.08061, 14.4101822]}
      zoom={16}
      scrollWheelZoom={true}
      className="w-screen h-screen diseable-map-selection"
    >
      <TileLayer
        url={`https://api.mapy.cz/v1/maptiles/outdoor/256/{z}/{x}/{y}?apikey=${MAP_API_KEY}`}
        attribution='<a href="https://api.mapy.cz/copyright" target="_blank" rel="noreferrer">&copy; Seznam.cz a.s. a další</a>'
      />
      <link
        rel="stylesheet"
        href="https://cdn.jsdelivr.net/npm/leaflet.locatecontrol@0.76.0/dist/L.Control.Locate.min.css"
      />
      <a
        href="http://mapy.cz/"
        target="_blank"
        rel="noreferrer"
        className="absolute z-[1000] bottom-0"
      >
        <img alt="Mapy.cz logo" src="https://api.mapy.cz/img/api/logo.svg" />
      </a>
    </MapContainer>
  );
}
