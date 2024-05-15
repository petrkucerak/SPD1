import { useMap } from "react-leaflet";

export default function Stations({ accuracy }) {
  const map = useMap();

  const url = `/api/stations?accuracy=${accuracy}`;

  fetch(url)
    .then((res) => res.json())
    .then((data) => {
      const stations = data.data;
      console.log(stations);
      stations.map((station) => {
        // prepare the output string
        const string = `
        <div class="card text-white">
          <div class="card-body">
            <h2 class="card-header">Stanice</h1>
            <p class="text-content2">${station.GN} ${station.GE}</p>
            <div class="card-footer">
              <a href='/station?GN=${station.GN}&GE=${station.GE}&accuracy=${accuracy}'>
                <button class="btn btn-solid-secondary font-semibold">Naměřená data</button>
              </a>
            </div>
          </div>
        </div>`;

        const statoinPopup = L.popup({
          keepInView: true,
          offset: L.point(20, 0),
          autoPanPadding: L.point(20, 20),
          closeButton: false,
          className: "",
        }).setContent(string);

        L.marker([station.GN, station.GE], {
          icon: stationIcon,
          title: "Stanice",
          alt: `Stanice`,
        })
          .addTo(map)
          .bindPopup(statoinPopup)
          .on("click", (e) => {
            // console.log(e); // TODO: remove
          });
      });
    })
    .catch((err) => console.log(err));

  return null;
}

const stationIcon = L.icon({
  iconUrl: "/station-icon.png",
  iconSize: [40, 40],
  iconAnchor: [20, 20],
  popupAnchor: [-20, -10],
  //   shadowUrl: "/station-icon.png",
  //   shadowSize: [35, 35],
  //   shadowAnchor: [0, 0],
});
