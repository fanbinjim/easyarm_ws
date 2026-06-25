export function NumberGrid({
  values,
  onChange,
  labels,
  step,
}: {
  values: number[];
  onChange: (value: number[]) => void;
  labels: string[];
  step: number;
}) {
  return (
    <div className="number-grid">
      {values.map((value, index) => (
        <label key={labels[index] ?? index}>
          <span>{labels[index] ?? index}</span>
          <input
            type="number"
            step={step}
            value={value}
            onChange={(event) => {
              const next = [...values];
              next[index] = Number(event.target.value);
              onChange(next);
            }}
          />
        </label>
      ))}
    </div>
  );
}
