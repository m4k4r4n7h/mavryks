import { useCallback, useEffect, useState } from "react"
import { post, remove } from "./api"
import type { Pair, Snapshot } from "./types"
import GridPanel from "./components/GridPanel"
import StatusPanel from "./components/StatusPanel"
import ControlPanel from "./components/ControlPanel"
import TaskPanel from "./components/TaskPanel"
import RobotPanel from "./components/RobotPanel"
import InstructionPanel from "./components/InstructionPanel"
import EventPanel from "./components/EventPanel"

const emptyState: Snapshot = {
  grid_size: 12,
  grid: [],
  rho: [],
  obstacles: [],
  dynamic_obstacles: [],
  static_obstacles: [],
  workers: [],
  robots: [],
  targets: [],
  path: [],
  steps: [],
  instructions: [],
  cost: 0,
  distance: 0,
  estimated_time_sec: 0,
  turns: 0,
  avg_congestion: 0,
  max_congestion: 0,
  reachable: false,
  reason: "no_targets",
  fleet_routes: [],
  commands: [],
  tasks: [],
  task_stats: {},
  safety: {
    estop: false,
    recommended_speed_mps: 0,
    commanded_speed_mps: 0,
    min_clearance_m: 0,
    brake_distance_m: 0,
    congestion_limit: 1,
    alerts: [],
  },
  quality: { score: 0, blur: 0, exposure: 0, glare: 0, shadow: 0, drift: 0, alerts: [] },
  proximity: [],
  estop: false,
  speed_limit_mps: 1.4,
  diagonal: true,
  config: {
    diagonal: true,
    cost_mode: "exact",
    congestion_weight: 4.5,
    turn_penalty: 0,
    clearance_weight: 0,
    clearance_radius: 2,
    cell_size_m: 1,
    worker_speed_mps: 1.4,
    optimize_order: false,
    snap_start_if_blocked: true,
    decel_mps2: 1.3,
  },
  vision: { online: false, fps: 0, last_update: null },
  metrics: {},
  diagnostics: {},
  events: [],
  updated_at: 0,
}

type TaskResult = { state: Snapshot }

export default function App() {
  const [state, setState] = useState<Snapshot>(emptyState)
  const [status, setStatus] = useState("DISCONNECTED")
  const [selected, setSelected] = useState<Pair | null>(null)

  const load = useCallback(async () => {
    try {
      const response = await fetch("/api/state")
      if (response.ok) {
        setState((await response.json()) as Snapshot)
      } else {
        setStatus("NO BACKEND")
      }
    } catch {
      setStatus("NO BACKEND")
    }
  }, [])

  useEffect(() => {
    let alive = true
    let timer: number | undefined
    let socket: WebSocket | null = null

    const connect = () => {
      const scheme = location.protocol === "https:" ? "wss:" : "ws:"
      socket = new WebSocket(`${scheme}//${location.host}/ws/live`)
      socket.onopen = () => setStatus("CONNECTED")
      socket.onclose = () => {
        setStatus("DISCONNECTED")
        if (alive) {
          timer = window.setTimeout(connect, 1500)
        }
      }
      socket.onerror = () => setStatus("SOCKET ERROR")
      socket.onmessage = (event) => {
        try {
          const msg = JSON.parse(event.data) as Snapshot
          if (msg && msg.grid && msg.grid_size) {
            setState(msg)
          }
        } catch {
          setStatus("BAD DATA")
        }
      }
    }

    void load()
    connect()
    return () => {
      alive = false
      if (timer !== undefined) {
        window.clearTimeout(timer)
      }
      socket?.close()
    }
  }, [load])

  function apply(next: Snapshot | null | undefined) {
    if (next && next.grid) {
      setState(next)
    }
  }

  function pick(x: number, y: number, append: boolean) {
    setSelected([x, y])
    void post<Snapshot>("/api/target", { x, y, append }).then(apply)
  }

  function zone(x: number, y: number) {
    void post<Snapshot>("/api/zones/static", { obstacles: [[x, y]], mode: "toggle" }).then(apply)
  }

  function setEstop(active: boolean) {
    void post<Snapshot>("/api/estop", { active }).then(apply)
  }

  function setSpeed(speed: number) {
    void post<Snapshot>("/api/speed", { speed_mps: speed }).then(apply)
  }

  function setConfig(patch: Record<string, boolean | string | number>) {
    void post<Snapshot>("/api/config", patch).then(apply)
  }

  function reset() {
    void post<Snapshot>("/api/reset", {}).then(apply)
  }

  function clearTargets() {
    void post<Snapshot>("/api/target", { targets: [] }).then(apply)
  }

  function clearZones() {
    void remove<Snapshot>("/api/zones/static").then(apply)
  }

  function addTask(kind: string, priority: number, payload: number) {
    if (!selected) {
      return
    }
    void post<TaskResult>("/api/tasks", {
      kind,
      x: selected[0],
      y: selected[1],
      priority,
      payload_kg: payload,
    }).then((result) => apply(result ? result.state : null))
  }

  function taskAction(id: string, action: "start" | "complete" | "fail" | "cancel") {
    void post<TaskResult>(`/api/tasks/${id}/${action}`, {}).then((result) =>
      apply(result ? result.state : null),
    )
  }

  function deleteTask(id: string) {
    void remove<Snapshot>(`/api/tasks/${id}`).then(apply)
  }

  function clearEvents() {
    void remove<Snapshot>("/api/events").then(apply)
  }

  return (
    <div className="app">
      <div className="bar">MAVRYKS PHYSICAL AI WAREHOUSE CONTROL | LIVE WEBCAM ONLY | {status}</div>
      <div className="main">
        <GridPanel state={state} selected={selected} onPick={pick} onZone={zone} />
        <div className="side">
          <StatusPanel state={state} status={status} />
          <ControlPanel
            state={state}
            onEstop={setEstop}
            onSpeed={setSpeed}
            onToggleDiagonal={() => setConfig({ diagonal: !state.config.diagonal })}
            onCostMode={(mode) => setConfig({ cost_mode: mode })}
            onOptimize={(value) => setConfig({ optimize_order: value })}
            onReset={reset}
            onClearTargets={clearTargets}
            onClearZones={clearZones}
          />
        </div>
      </div>
      <div className="lower">
        <TaskPanel
          state={state}
          selected={selected}
          onAdd={addTask}
          onAction={taskAction}
          onDelete={deleteTask}
        />
        <RobotPanel state={state} />
      </div>
      <InstructionPanel state={state} />
      <EventPanel state={state} onClear={clearEvents} />
    </div>
  )
}
