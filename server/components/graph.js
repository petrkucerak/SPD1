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

export default function Graph({ data }) {
  ChartJS.register(CategoryScale /* ... */);

  return (
    <div>
      <Line
        options={{
          responsive: true,
          plugins: {
            title: {
              display: true,
              text: "Chart.js Line Chart - Cubic interpolation mode",
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
                text: "Value",
              },
              suggestedMin: -10,
              suggestedMax: 200,
            },
          },
        }}
        data={{
          labels: [
            "2023-01",
            "2023-02",
            "2023-03",
            "2023-04",
            "2023-05",
            "2023-06",
            "2023-07",
          ],
          datasets: [
            {
              label: "Cubic interpolation",
              data: [100, 120, 115, 134, 168, 132, 200],
              backgroundColor: "red",
              borderColor: "red",
              fill: true,
              cubicInterpolationMode: "monotone",
              tension: 0.4,
            },
          ],
        }}
      />
    </div>
  );
}
