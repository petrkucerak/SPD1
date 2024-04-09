"use client";

import Map, { GeolocateControl } from "react-map-gl";
import "mapbox-gl/dist/mapbox-gl.css";

export default function MapComponent() {
  return (
    <div>
      <Map
        mapboxAccessToken="pk.eyJ1Ijoia3VjZXJwMjgiLCJhIjoiY2x1c3lkd3liMDc3NDJpbzFmdHRna2YxYyJ9.Fw0lru7f7Er2Dckyd-r4UQ"
        initialViewState={{
          longitude: -100,
          latitude: 40,
          zoom: 3.5,
        }}
        mapStyle="mapbox://styles/mapbox/streets-v11"
      >
        <GeolocateControl
          positionOptions={{ enableHighAccuracy: true }}
          trackUserLocation={true}
        />
      </Map>
    </div>
  );
}
