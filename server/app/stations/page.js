import dynamic from "next/dynamic";

// const Map = dynamic(() => import("@/components/map"), {
//   loading: () => <p>Načítání</p>,
//   ssr: false,
// });

export default function Stations() {
  return (
    <h1>Stanice</h1>
    // <Map />
  );
}
