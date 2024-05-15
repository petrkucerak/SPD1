export default function Diary() {
  return (
    <div className="flex flex-col w-[90vw] max-w-[800px] justify-around">
      <h1 className="text-2xl font-semibold">Deníček</h1>
      <p>
        Deníček je místo, kam si můžeš zapisovat, jak se pod nešním otužování
        cítíš a sledovat tak svůj posun v čase.
      </p>
      <div className="flex sm:flex-row justify-around mt-8 flex-col">
        <button className="btn btn-solid-primary btn-xl my-2">
          Zapsat dnešní den
        </button>

        <button className="btn btn-solid-secondary btn-xl my-2">
          Prohlédnout výsledky
        </button>
      </div>
    </div>
  );
}
