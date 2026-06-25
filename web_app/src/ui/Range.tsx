export function Range({
  label,
  value,
  setValue,
}: {
  label: string;
  value: number;
  setValue: (value: number) => void;
}) {
  return (
    <label className="range-row">
      <span>{label}</span>
      <input type="range" min="0.01" max="1" step="0.01" value={value} onChange={(event) => setValue(Number(event.target.value))} />
      <strong>{value.toFixed(2)}</strong>
    </label>
  );
}
