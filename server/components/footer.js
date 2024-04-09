import Link from "next/link";

export default function Footer() {
  return (
    <div className="navbar text-sm">
      <div className="navbar-start">
        <Link href={`/`} className="navbar-item font-bold uppercase">
          💪&nbsp;Otužilcův&nbsp;deníček
        </Link>
      </div>
      <div className="navbar-end">
        <Link
          href={`https://github.com/petrkucerak/weather-station-with-geolocation`}
          target="_blank"
          className="navbar-item"
        >
          Repositář projektu
        </Link>
        <Link href={`/about`} className="navbar-item">
          O projektu
        </Link>
      </div>
    </div>
  );
}
