export async function post<T>(path: string, body: unknown): Promise<T | null> {
  try {
    const r = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
    })
    if (!r.ok) {
      return null
    }
    return (await r.json()) as T
  } catch {
    return null
  }
}

export async function remove<T>(path: string): Promise<T | null> {
  try {
    const r = await fetch(path, { method: "DELETE" })
    if (!r.ok) {
      return null
    }
    return (await r.json()) as T
  } catch {
    return null
  }
}
