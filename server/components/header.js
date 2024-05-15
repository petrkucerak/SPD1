import Link from "next/link";

export default function Header() {
  return (
    <div className="navbar">
      <div className="navbar-start">
        <Link href={`/`} className="navbar-item font-bold uppercase">
          💪&nbsp;Otužilcův&nbsp;deníček
        </Link>
      </div>
      <div className="navbar-end">
        <Link href={`/diary`} className="navbar-item">
          Deníček
        </Link>
        <Link href={`/map`} className="navbar-item">
          Stanice
        </Link>
      </div>
    </div>
  );
}
