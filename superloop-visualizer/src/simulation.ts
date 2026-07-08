export type TaskId = 'lidar' | 'imu'

export type TaskConfig = {
  id: TaskId
  label: string
  shortLabel: string
  c: number
  t: number
  d: number
  initialRelease: number
}

export type ReleaseStatus = 'future' | 'released' | 'done' | 'miss' | 'skipped'

export type Release = {
  id: string
  taskId: TaskId
  time: number
  index: number
  status: ReleaseStatus
}

export type ExecutionBlockStatus = 'running' | 'ok' | 'miss'

export type ExecutionBlock = {
  id: string
  taskId: TaskId
  label: string
  releaseTime: number
  start: number
  finish: number
  response: number | null
  status: ExecutionBlockStatus
}

export type TaskStats = {
  runs: number
  misses: number
  skips: number
}

export type LastResponse = {
  label: string
  taskId: TaskId
  response: number
  deadline: number
  missed: boolean
} | null

export type SimulationState = {
  time: number
  loopPass: number
  phase: StepKind
  nextRelease: Record<TaskId, number>
  stats: Record<TaskId, TaskStats>
  releases: Release[]
  execution: ExecutionBlock[]
  currentBlockId: string | null
  lastResponse: LastResponse
}

export type StepKind =
  | 'intro'
  | 'decision'
  | 'start'
  | 'finish'
  | 'advance'
  | 'skip'
  | 'idle'
  | 'summary'

export type SimulationStep = {
  title: string
  body: string
  kind: StepKind
  snapshot: SimulationState
}

export const scenario = {
  windowMs: 100,
  order: ['lidar', 'imu'] as TaskId[],
  tasks: [
    {
      id: 'lidar',
      label: 'LiDAR',
      shortLabel: 'L',
      c: 20,
      t: 50,
      d: 50,
      initialRelease: 0,
    },
    {
      id: 'imu',
      label: 'IMU',
      shortLabel: 'I',
      c: 10,
      t: 20,
      d: 20,
      initialRelease: 0,
    },
  ] satisfies TaskConfig[],
}

export const taskMap = Object.fromEntries(
  scenario.tasks.map((task) => [task.id, task]),
) as Record<TaskId, TaskConfig>

function cloneState(state: SimulationState): SimulationState {
  return structuredClone(state)
}

function createReleases(): Release[] {
  return scenario.tasks.flatMap((task) => {
    const releases: Release[] = []
    let index = 1

    for (
      let time = task.initialRelease;
      time <= scenario.windowMs;
      time += task.t
    ) {
      releases.push({
        id: `${task.id}-${time}`,
        taskId: task.id,
        time,
        index,
        status: 'future',
      })
      index += 1
    }

    return releases
  })
}

function createInitialState(): SimulationState {
  const state: SimulationState = {
    time: 0,
    loopPass: 0,
    phase: 'intro',
    nextRelease: {
      lidar: 0,
      imu: 0,
    },
    stats: {
      lidar: { runs: 0, misses: 0, skips: 0 },
      imu: { runs: 0, misses: 0, skips: 0 },
    },
    releases: createReleases(),
    execution: [],
    currentBlockId: null,
    lastResponse: null,
  }

  updateOccurredReleases(state)

  return state
}

function updateOccurredReleases(state: SimulationState) {
  for (const release of state.releases) {
    if (release.time <= state.time && release.status === 'future') {
      release.status = 'released'
    }
  }
}

function markRelease(
  state: SimulationState,
  taskId: TaskId,
  releaseTime: number,
  status: ReleaseStatus,
) {
  const release = state.releases.find(
    (item) => item.taskId === taskId && item.time === releaseTime,
  )

  if (release) {
    release.status = status
  }
}

function pushStep(
  steps: SimulationStep[],
  state: SimulationState,
  kind: StepKind,
  title: string,
  body: string,
) {
  state.phase = kind
  steps.push({
    title,
    body,
    kind,
    snapshot: cloneState(state),
  })
}

function getReleaseLabel(
  state: SimulationState,
  taskId: TaskId,
  releaseTime: number,
) {
  const release = state.releases.find(
    (item) => item.taskId === taskId && item.time === releaseTime,
  )

  return `R${release?.index ?? '?'}`
}

export function simulateSuperloop(): SimulationStep[] {
  const state = createInitialState()
  const steps: SimulationStep[] = []

  pushStep(
    steps,
    state,
    'intro',
    'Initial release',
    'Ready: LiDAR R1, IMU R1',
  )

  while (state.time < scenario.windowMs) {
    updateOccurredReleases(state)

    const readyFlags = Object.fromEntries(
      scenario.tasks.map((task) => [
        task.id,
        state.time >= state.nextRelease[task.id],
      ]),
    ) as Record<TaskId, boolean>

    const releaseSnapshot = Object.fromEntries(
      scenario.tasks.map((task) => [task.id, state.nextRelease[task.id]]),
    ) as Record<TaskId, number>

    const readyTasks = scenario.tasks.filter((task) => readyFlags[task.id])

    if (readyTasks.length === 0) {
      state.loopPass += 1
      const nextTime = Math.min(
        scenario.windowMs,
        ...scenario.tasks.map((task) => state.nextRelease[task.id]),
      )

      state.time = nextTime
      updateOccurredReleases(state)

      pushStep(
        steps,
        state,
        'idle',
        `Polling until ${state.time} ms`,
        'No task is ready',
      )

      continue
    }

    state.loopPass += 1

    pushStep(
      steps,
      state,
      'decision',
      `Superloop check`,
      `Ready: ${readyTasks.map((task) => task.label).join(', ')}`,
    )

    for (const taskId of scenario.order) {
      if (!readyFlags[taskId] || state.time >= scenario.windowMs) {
        continue
      }

      const task = taskMap[taskId]
      const releaseTime = releaseSnapshot[taskId]
      const releaseLabel = getReleaseLabel(state, taskId, releaseTime)
      const jobLabel = `${task.label} ${releaseLabel}`
      const start = state.time
      const block: ExecutionBlock = {
        id: `${task.id}-${releaseTime}-${start}`,
        taskId,
        label: jobLabel,
        releaseTime,
        start,
        finish: start + task.c,
        response: null,
        status: 'running',
      }

      state.execution.push(block)
      state.currentBlockId = block.id

      pushStep(
        steps,
        state,
        'start',
        `Run ${jobLabel}`,
        `Start ${start} ms, execution ${task.c} ms, finish ${block.finish} ms`,
      )

      state.time = block.finish
      updateOccurredReleases(state)

      const response = state.time - releaseTime
      const missed = response > task.d

      block.response = response
      block.status = missed ? 'miss' : 'ok'
      state.currentBlockId = null
      state.stats[taskId].runs += 1
      state.lastResponse = {
        label: jobLabel,
        taskId,
        response,
        deadline: task.d,
        missed,
      }

      if (missed) {
        state.stats[taskId].misses += 1
      }

      markRelease(state, taskId, releaseTime, missed ? 'miss' : 'done')

      pushStep(
        steps,
        state,
        'finish',
        `${jobLabel}: ${missed ? 'MISS' : 'OK'}`,
        `Response ${state.time}-${releaseTime}=${response} ms, deadline ${task.d} ms`,
      )

      state.nextRelease[taskId] += task.t

      const skipped: number[] = []

      while (state.time >= state.nextRelease[taskId]) {
        const skippedTime = state.nextRelease[taskId]
        skipped.push(skippedTime)
        markRelease(state, taskId, skippedTime, 'skipped')
        state.stats[taskId].skips += 1
        state.nextRelease[taskId] += task.t
      }

      if (skipped.length > 0) {
        pushStep(
          steps,
          state,
          'skip',
          `Skip ${task.label} release`,
          `t_cur ${state.time} ms > release ${skipped.join(', ')} ms. Next: ${state.nextRelease[taskId]} ms`,
        )
      } else {
        pushStep(
          steps,
          state,
          'advance',
          `Update ${task.label}`,
          `Next release: ${state.nextRelease[taskId]} ms`,
        )
      }
    }
  }

  updateOccurredReleases(state)

  pushStep(
    steps,
    state,
    'summary',
    't=100: window edge',
    'next window starts here',
  )

  return steps
}
