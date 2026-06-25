import { useRef } from "react";
import { Play, Pause } from "lucide-react";

export function StreamButtons({ onStart, onStop }: { onStart: () => void; onStop: () => void }) {
  const timerRef = useRef<number | null>(null);

  const start = () => {
    onStart();
    if (timerRef.current !== null) window.clearInterval(timerRef.current);
    timerRef.current = window.setInterval(onStart, 100);
  };

  const stop = () => {
    if (timerRef.current !== null) {
      window.clearInterval(timerRef.current);
      timerRef.current = null;
    }
    onStop();
  };

  return (
    <div className="stream-buttons">
      <button onMouseDown={start} onMouseUp={stop} onMouseLeave={stop} onTouchStart={start} onTouchEnd={stop}>
        <Play /> 按住发送
      </button>
      <button className="ghost-button" onClick={stop}>
        <Pause /> Halt
      </button>
    </div>
  );
}
