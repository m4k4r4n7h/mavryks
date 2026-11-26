import type { Snapshot } from "../types"

interface Props {
  state: Snapshot
}

function fmt(p: [number, number]) {
  return `(${p[0]},${p[1]})`
}

export default function InstructionPanel({ state }: Props) {
  const instructions = state.instructions || []
  const steps = state.steps || []

  return (
    <section className="panel">
      <h2>ROUTE INSTRUCTIONS</h2>
      <div className="twoCol">
        <div>
          <h3>COMPRESSED DIRECTIONS</h3>
          <div className="tableWrap short">
            <table className="steps">
              <thead>
                <tr>
                  <th>#</th>
                  <th>ACTION</th>
                  <th>CELLS</th>
                  <th>METERS</th>
                  <th>FROM</th>
                  <th>TO</th>
                </tr>
              </thead>
              <tbody>
                {instructions.map((item, index) => (
                  <tr key={index}>
                    <td>{index + 1}</td>
                    <td>{item.action}</td>
                    <td>{item.cells}</td>
                    <td>{item.distance_m}</td>
                    <td>{fmt(item.from)}</td>
                    <td>{fmt(item.to)}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
        <div>
          <h3>EXACT A* STEPS</h3>
          <div className="tableWrap short">
            <table className="steps">
              <thead>
                <tr>
                  <th>STEP</th>
                  <th>FROM</th>
                  <th>TO</th>
                  <th>DIR</th>
                  <th>DIST</th>
                  <th>RHO</th>
                  <th>COST</th>
                </tr>
              </thead>
              <tbody>
                {steps.map((step) => (
                  <tr key={step.step}>
                    <td>{step.step}</td>
                    <td>{fmt(step.from)}</td>
                    <td>{fmt(step.to)}</td>
                    <td>{step.dir}</td>
                    <td>{step.distance === undefined ? "1" : step.distance}</td>
                    <td>{step.congestion === undefined ? "0" : step.congestion}</td>
                    <td>{step.cost.toFixed(4)}</td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        </div>
      </div>
      <div className="note">COST = d(u,v) * (1 + 4.5 * rho_v^2) + 10^6 * I_blocked(v)</div>
    </section>
  )
}
