import Link from "next/link";
import IconGitHub from "./icons/github";

export default function Footer() {
  return (
    <div className="navbar text-sm">
      <div className="navbar-start">
        <Link href={`/`} className="navbar-item font-bold uppercase">
          💪&nbsp;Otužilcův&nbsp;deníček
        </Link>
      </div>
      <div className="navbar-end">
        <Link href={`/about`} className="navbar-item">
          O projektu
        </Link>
        <Link
          href={`https://github.com/petrkucerak/weather-station-with-geolocation`}
          target="_blank"
          className="navbar-item"
        >
          <IconGitHub
            stroke={`#fff`}
            strokeWidth={1.8}
            width={20}
            height={20}
            className={"inline-block mr-1"}
          />
        </Link>
      </div>
    </div>
  );
}
