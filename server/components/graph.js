// components/MyLineChart.tsx
"use client";
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  Tooltip,
  PointElement,
  LineElement,
} from "chart.js";
import { Line } from "react-chartjs-2";
import gpsToUtc from "./gps2utc";

// Register ChartJS components using ChartJS.register
ChartJS.register(
  CategoryScale,
  LinearScale,
  PointElement,
  LineElement,
  Tooltip
);

export default function Graph({ data, type, label, title }) {
  ChartJS.register(CategoryScale /* ... */);

  return (
    <div>
      <Line
        options={{
          responsive: true,
          plugins: {
            title: {
              display: true,
              text: title != undefined ? title : "title",
            },
          },
          interaction: {
            intersect: false,
          },
          scales: {
            x: {
              display: true,
              title: {
                display: true,
              },
            },
            y: {
              display: true,
              title: {
                display: true,
                text: label != undefined ? label : "value",
              },
              suggestedMin: -10,
              suggestedMax: 200,
            },
          },
        }}
        data={{
          labels: data
            .map((item) => /*gpsToUtc(item.gps_time)*/ item.gps_time)
            .slice(-24),

          datasets:
            type === "light"
              ? [
                  {
                    label: "Světelnost",
                    data: data.map((item) => item.Lo).slice(-24),
                    backgroundColor: "red",
                    borderColor: "red",
                    fill: true,
                    cubicInterpolationMode: "monotone",
                    tension: 0.4,
                  },
                ]
              : type === "temperature"
              ? [
                  {
                    label: "Teplota vody",
                    data: data.map((item) => item.Tw).slice(-24),
                    backgroundColor: "blue",
                    borderColor: "blue",
                    fill: true,
                    cubicInterpolationMode: "monotone",
                    tension: 0.4,
                  },
                ]
              : [],
        }}
      />
    </div>
  );
}
