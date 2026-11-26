import { useState } from "react"
import type { Pair, Snapshot } from "../types"

interface Props {
  state: Snapshot
  selected: Pair | null
  onAdd: (kind: string, priority: number, payload: number) => void
  onAction: (id: string, action: "start" | "complete" | "fail" | "cancel") => void
  onDelete: (id: string) => void
}

const kinds = ["pick", "place", "dock", "inspect", "charge"]

export default function TaskPanel({ state, selected, onAdd, onAction, onDelete }: Props) {
  const [kind, setKind] = useState("pick")
  const [priority, setPriority] = useState(0)
  const [payload, setPayload] = useState(0)
  const stats = state.task_stats
  const target = selected ? `(${selected[0]},${selected[1]})` : "NONE"
  const summary = kinds.map((k) => `${k.toUpperCase()} ${state.tasks.filter((t) => t.status !== "done" && t.status !== "failed" && t.status !== "cancelled" && t.kind === k).length}`).join(" | ")

  return (
    <section className="panel">
      <h2>WAREHOUSE TASK QUEUE</h2>
      <div className="note">
        PENDING {stats.pending || 0} | ASSIGNED {stats.assigned || 0} | ACTIVE {stats.active || 0} | DONE {stats.done || 0} | FAILED {stats.failed || 0} | CANCELLED {stats.cancelled || 0}
      </div>
      <div className="note">{summary}</div>
      <div className="row">
        <select className="field" value={kind} onChange={(event) => setKind(event.target.value)}>
          {kinds.map((k) => (
            <option key={k} value={k}>{k.toUpperCase()}</option>
          ))}
        </select>
        <span>TARGET {target}</span>
        <input
          className="field num"
          type="number"
          min={-20}
          max={20}
          value={priority}
          onChange={(event) => setPriority(Number(event.target.value))}
          aria-label="Priority"
        />
        <input
          className="field num"
          type="number"
          min={0}
          max={1000}
          step={0.5}
          value={payload}
          onChange={(event) => setPayload(Number(event.target.value))}
          aria-label="Payload kilograms"
        />
        <button className="btn" disabled={!selected} onClick={() => onAdd(kind, priority, payload)}>QUEUE TASK</button>
      </div>
      <div className="tableWrap short">
        <table className="steps">
          <thead>
            <tr>
              <th>ID</th>
              <th>KIND</th>
              <th>TARGET</th>
              <th>PRIO</th>
              <th>ROBOT</th>
              <th>KG</th>
              <th>STATUS</th>
              <th>ACTIONS</th>
            </tr>
          </thead>
          <tbody>
            {state.tasks.map((task) => (
              <tr key={task.id}>
                <td>{task.id}</td>
                <td>{task.kind.toUpperCase()}</td>
                <td>({task.target[0]},{task.target[1]})</td>
                <td>{task.priority}</td>
                <td>{task.robot_id === null ? "-" : `R${task.robot_id}`}</td>
                <td>{task.payload_kg}</td>
                <td>{task.status.toUpperCase()}</td>
                <td>
                  <button className="btn small" onClick={() => onAction(task.id, "start")}>START</button>
                  <button className="btn small" onClick={() => onAction(task.id, "complete")}>DONE</button>
                  <button className="btn small" onClick={() => onAction(task.id, "fail")}>FAIL</button>
                  <button className="btn small" onClick={() => onAction(task.id, "cancel")}>CANCEL</button>
                  <button className="btn small" onClick={() => onDelete(task.id)}>DEL</button>
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
      <div className="note">CLICK A GRID CELL TO SELECT A TARGET, THEN QUEUE A TASK</div>
    </section>
  )
}
