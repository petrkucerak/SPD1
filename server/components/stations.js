import { useMap } from "react-leaflet";

export default function Stations() {
  const map = useMap();

  // prepare the output string
  const string = `Stanice 1
`;

  const statoinPopup = L.popup({
    keepInView: true,
    offset: L.point(15, 0),
    autoPanPadding: L.point(15, 15),
    closeButton: false,
    className: "",
  }).setContent(string);

  L.marker([50.08061, 14.4101822], {
    icon: stationIcon,
    title: "Stanice",
    alt: `Stanice`,
  })
    .addTo(map)
    .bindPopup(statoinPopup)
    .on("click", (e) => {
      console.log(e);
    });

  return null;
}

const stationIcon = L.icon({
  iconUrl: "/station-icon.png",
  iconSize: [40, 40],
  iconAnchor: [0, 0],
  popupAnchor: [0, 0],
//   shadowUrl: "/station-icon.png",
//   shadowSize: [35, 35],
//   shadowAnchor: [0, 0],
});
