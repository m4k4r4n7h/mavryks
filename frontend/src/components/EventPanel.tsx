import type { Snapshot } from "../types"

interface Props {
  state: Snapshot
  onClear: () => void
}

export default function EventPanel({ state, onClear }: Props) {
  const events = state.events || []

  return (
    <section className="panel">
      <h2>EVENT LOG <button className="btn small" onClick={onClear}>CLEAR</button></h2>
      <div className="tableWrap short">
        <table className="steps">
          <thead>
            <tr>
              <th>TIME</th>
              <th>LEVEL</th>
              <th>EVENT</th>
              <th>DATA</th>
            </tr>
          </thead>
          <tbody>
            {events.map((event) => (
              <tr key={event.id}>
                <td>{new Date(event.ts * 1000).toLocaleTimeString()}</td>
                <td>{event.severity.toUpperCase()}</td>
                <td>{event.message}</td>
                <td>{JSON.stringify(event.data)}</td>
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </section>
  )
}
