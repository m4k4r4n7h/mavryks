export type Pair = [number, number]

export interface Step {
  step: number
  from: Pair
  to: Pair
  dir: string
  cost: number
  distance?: number
  congestion?: number
}

export interface Instruction {
  action: string
  dir: string
  cells: number
  distance_m: number
  from: Pair
  to: Pair
}

export interface Route {
  worker_id?: number
  path: Pair[]
  steps: Step[]
  instructions: Instruction[]
  cost: number
  distance: number
  estimated_time_sec: number
  turns: number
  reachable: boolean
  reason: string
}

export interface Task {
  id: string
  kind: string
  target: Pair
  priority: number
  robot_id: number | null
  payload_kg: number
  label: string
  status: string
  created_at: number
  started_at: number | null
  finished_at: number | null
  reason: string
}

export interface Robot {
  robot_id: number
  cell: Pair
  pos?: Pair
  heading_deg: number
  speed_mps: number
  battery_pct: number
  source: string
}

export interface Command {
  robot_id: number | string
  linear_mps?: number
  angular_radps?: number
  left_wheel_radps?: number
  right_wheel_radps?: number
  brake_distance_m?: number
  cross_track_m?: number
  arrival?: boolean
  utilization?: number
  battery_drain_pct?: number
}

export interface Safety {
  estop: boolean
  recommended_speed_mps: number
  commanded_speed_mps: number
  min_clearance_m: number
  brake_distance_m: number
  congestion_limit: number
  alerts: string[]
}

export interface Quality {
  score: number
  blur: number
  exposure: number
  glare: number
  shadow: number
  drift: number
  alerts: string[]
}

export interface EventItem {
  id: number
  ts: number
  severity: string
  message: string
  data: Record<string, unknown>
}

export interface Snapshot {
  grid_size: number
  grid: number[][]
  rho: number[][]
  obstacles: Pair[]
  dynamic_obstacles: Pair[]
  static_obstacles: Pair[]
  workers: Pair[]
  robots: Robot[]
  targets: Pair[]
  path: Pair[]
  steps: Step[]
  instructions: Instruction[]
  cost: number
  distance: number
  estimated_time_sec: number
  turns: number
  avg_congestion: number
  max_congestion: number
  reachable: boolean
  reason: string
  fleet_routes: Route[]
  commands: Command[]
  tasks: Task[]
  task_stats: Record<string, number>
  safety: Safety
  quality: Quality
  proximity: Array<Record<string, unknown>>
  estop: boolean
  speed_limit_mps: number
  diagonal: boolean
  config: {
    diagonal: boolean
    cost_mode: string
    congestion_weight: number
    turn_penalty: number
    clearance_weight: number
    clearance_radius: number
    cell_size_m: number
    worker_speed_mps: number
    optimize_order: boolean
    snap_start_if_blocked: boolean
    decel_mps2: number
  }
  vision: { online: boolean; fps: number; last_update: number | null }
  metrics: Record<string, number>
  diagnostics: Record<string, number>
  events: EventItem[]
  updated_at: number
}
