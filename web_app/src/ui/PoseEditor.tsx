export type PoseValues = {
  x: number;
  y: number;
  z: number;
  qx: number;
  qy: number;
  qz: number;
  qw: number;
};

export const DEFAULT_MOVEL: PoseValues = { x: 0.25, y: 0, z: 0.25, qx: 0, qy: 0, qz: 0, qw: 1 };

export function PoseEditor({
  value,
  onChange,
}: {
  value: PoseValues;
  onChange: (value: PoseValues) => void;
}) {
  const fields: Array<keyof PoseValues> = ["x", "y", "z", "qx", "qy", "qz", "qw"];
  return (
    <div className="number-grid pose-grid">
      {fields.map((field) => (
        <label key={field}>
          <span>{field}</span>
          <input
            type="number"
            step={field.startsWith("q") ? 0.01 : 0.005}
            value={value[field]}
            onChange={(event) => onChange({ ...value, [field]: Number(event.target.value) })}
          />
        </label>
      ))}
    </div>
  );
}
