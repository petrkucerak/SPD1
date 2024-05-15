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

  console.log(data);

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
            },
          },
        }}
        data={{
          labels: data
            .map(
              (item) =>
                `${new Date(item.time).getDate()}. ${
                  new Date(item.time).getMonth() + 1
                }. ${new Date(item.time).getHours()}:${new Date(
                  item.time
                ).getMinutes()}`
            )
            .slice(-48),

          datasets: [
            type.includes("Lo")
              ? {
                  label: "Světelnost",
                  data: data.map((item) => item.Lo).slice(-24),
                  backgroundColor: "red",
                  borderColor: "red",
                  fill: true,
                  cubicInterpolationMode: "monotone",
                  tension: 0.4,
                }
              : {},
            type.includes("Tw")
              ? {
                  label: "Teplota vody",
                  data: data.map((item) => item.Tw).slice(-24),
                  backgroundColor: "blue",
                  borderColor: "blue",
                  fill: true,
                  cubicInterpolationMode: "monotone",
                  tension: 0.4,
                }
              : {},
            type.includes("Ta")
              ? {
                  label: "Teplota vzduchu",
                  data: data.map((item) => item.Ta).slice(-24),
                  backgroundColor: "yellow",
                  borderColor: "yellow",
                  fill: true,
                  cubicInterpolationMode: "monotone",
                  tension: 0.4,
                }
              : {},
            type.includes("Ma")
              ? {
                  label: "Teplota vzduchu",
                  data: data.map((item) => item.Ma).slice(-24),
                  backgroundColor: "violet",
                  borderColor: "violet",
                  fill: true,
                  cubicInterpolationMode: "monotone",
                  tension: 0.4,
                }
              : {},
          ],
        }}
      />
    </div>
  );
}
