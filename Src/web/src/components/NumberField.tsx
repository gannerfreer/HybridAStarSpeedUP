import { useEffect, useState } from 'react';

/** 数字输入（失焦/回车提交；非法输入回退显示值）。W2/W3 面板共用。 */
export function NumberField(props: {
  label: string;
  value: number;
  step?: number;
  disabled?: boolean;
  onCommit: (value: number) => void;
}) {
  const [text, setText] = useState(String(props.value));
  useEffect(() => {
    setText(String(props.value));
  }, [props.value]);
  const commit = () => {
    const parsed = Number(text);
    if (Number.isFinite(parsed)) props.onCommit(parsed);
    else setText(String(props.value));
  };
  return (
    <label className="field">
      <span>{props.label}</span>
      <input
        type="number"
        step={props.step ?? 'any'}
        value={text}
        disabled={props.disabled}
        onChange={(event) => setText(event.target.value)}
        onBlur={commit}
        onKeyDown={(event) => {
          if (event.key === 'Enter') commit();
        }}
      />
    </label>
  );
}
