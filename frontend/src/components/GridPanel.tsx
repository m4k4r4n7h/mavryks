import type { MouseEvent, ReactNode } from "react"
import type { Pair, Snapshot } from "../types"

interface Props {
  state: Snapshot
  selected: Pair | null
  onPick: (x: number, y: number, append: boolean) => void
  onZone: (x: number, y: number) => void
}

const routeColors = ["#0033CC", "#008000", "#800080", "#CC6600", "#666666"]

function show(p: Pair) {
  return `(${p[0]},${p[1]})`
}

export default function GridPanel({ state, selected, onPick, onZone }: Props) {
  const n = state.grid_size || state.grid.length || 12
  const cells: ReactNode[] = []
  for (let y = 0; y < n; y++) {
    for (let x = 0; x < n; x++) {
      const blocked = state.grid[y]?.[x] === 1
      const rv = state.rho[y]?.[x] ?? 0
      const shade = Math.round(255 - 55 * Math.min(1, Math.max(0, rv)))
      const fill = blocked ? "#C00000" : `rgb(${shade},${shade},${shade})`
      cells.push(
        <rect
          key={`${x}-${y}`}
          className="cell"
          x={x}
          y={y}
          width={1}
          height={1}
          fill={fill}
          stroke="#808080"
          strokeWidth={0.04}
          onClick={(event: MouseEvent<SVGRectElement>) => onPick(x, y, event.shiftKey)}
          onContextMenu={(event: MouseEvent<SVGRectElement>) => {
            event.preventDefault()
            onZone(x, y)
          }}
        />,
      )
    }
  }

  const routes = state.fleet_routes?.length
    ? state.fleet_routes
    : state.path.length
      ? [{ path: state.path, worker_id: 0 }]
      : []

  return (
    <section className="panel">
      <h2>LIVE FLOOR GRID</h2>
      <svg className="grid" viewBox={`${-0.6} ${-0.6} ${n + 1.2} ${n + 1.2}`} role="grid">
        {cells}
        {selected && (
          <rect
            x={selected[0] + 0.04}
            y={selected[1] + 0.04}
            width={0.92}
            height={0.92}
            fill="none"
            stroke="#000000"
            strokeWidth={0.1}
          />
        )}
        {routes.map((route, idx) =>
          route.path.length > 1 ? (
            <polyline
              key={`route-${idx}`}
              points={route.path.map((p) => `${p[0] + 0.5},${p[1] + 0.5}`).join(" ")}
              fill="none"
              stroke={routeColors[idx % routeColors.length]}
              strokeWidth={0.16}
              strokeLinejoin="miter"
              strokeLinecap="butt"
            />
          ) : null,
        )}
        {state.targets.map((p, i) => (
          <rect
            key={`target-${i}`}
            x={p[0] + 0.12}
            y={p[1] + 0.12}
            width={0.76}
            height={0.76}
            fill="none"
            stroke="#0033CC"
            strokeWidth={0.12}
          />
        ))}
        {state.workers.map((p, i) => (
          <circle key={`worker-${i}`} cx={p[0] + 0.5} cy={p[1] + 0.5} r={0.24} fill="#000000" />
        ))}
        {state.robots.map((r, i) => (
          <g key={`robot-${i}`}>
            <rect x={r.cell[0] + 0.18} y={r.cell[1] + 0.18} width={0.64} height={0.64} fill="#FFFFFF" stroke="#000000" strokeWidth={0.08} />
            <text x={r.cell[0] + 0.5} y={r.cell[1] + 0.66} textAnchor="middle" fontSize={0.38} fontFamily="Courier New">
              R{r.robot_id}
            </text>
          </g>
        ))}
      </svg>
      <div className="note">LEFT CLICK SET TARGET | SHIFT CLICK APPEND | RIGHT CLICK STATIC ZONE</div>
      <div className="note">SELECTED {selected ? show(selected) : "NONE"} | ROBOTS {state.robots.length} | TARGETS {state.targets.length}</div>
    </section>
  )
}
