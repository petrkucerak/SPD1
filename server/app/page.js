import Link from "next/link";

export default function Home() {
  return (
    <div className="flex flex-row w-[90vw] max-w-[800px] justify-around">
      <Link href="/map">
        <button className="btn btn-solid-primary btn-xl">Stanice</button>
      </Link>
      <Link href="/">
        <button className="btn btn-solid-secondary btn-xl">Deníček</button>
      </Link>
    </div>
  );
}
