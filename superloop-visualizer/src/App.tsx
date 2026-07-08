import { useEffect, useMemo, useRef, useState } from 'react';
import './App.css';
import {
  scenario,
  simulateSuperloop,
  taskMap,
  type ExecutionBlock,
  type Release,
  type ReleaseStatus,
  type SimulationStep,
  type StepKind,
  type TaskId,
} from './simulation';

const SIM_MS_PER_SECOND = 12;

const timeline = {
  viewBoxWidth: 1120,
  viewBoxHeight: 440,
  left: 118,
  right: 1060,
  releaseAxisY: 142,
  executionAxisY: 316,
};

const taskColors: Record<TaskId, string> = {
  lidar: '#ff4d6d',
  imu: '#00c2ff',
};

const phaseOrder: {
  id: StepKind;
  label: string;
  hint: string;
  core: string;
}[] = [
  { id: 'decision', label: 'CHECK', hint: 'ready?', core: 'CHECK READY' },
  { id: 'start', label: 'RUN', hint: 'non-preemptive', core: 'RUN TASK' },
  { id: 'finish', label: 'DEADLINE', hint: 'R <= D', core: 'CHECK DEADLINE' },
  { id: 'skip', label: 'SKIP', hint: 'missed releases', core: 'COUNT SKIPS' },
  { id: 'advance', label: 'NEXT', hint: 'next release', core: 'UPDATE NEXT' },
  { id: 'idle', label: 'POLL', hint: 'no task ready', core: 'WAIT RELEASE' },
];

const phaseNotes: Record<
  StepKind,
  {
    title: string;
    accent: string;
  }
> = {
  intro: {
    title: 'Initial release',
    accent: 'Both tasks are ready at t=0. No job queue is stored.',
  },
  decision: {
    title: 'Ready test',
    accent: 'Ready means t_cur reached next_release.',
  },
  start: {
    title: 'Non-preemptive run',
    accent: 'The task owns the CPU until C is finished.',
  },
  finish: {
    title: 'Response check',
    accent: 'Response = finish time - release time.',
  },
  advance: {
    title: 'next_release update',
    accent: 'Only this task moves to its next period.',
  },
  skip: {
    title: 'Skipped release',
    accent: 'Past releases are counted, then next_release jumps.',
  },
  idle: {
    title: 'Polling gap',
    accent: 'No ready task. Time jumps to the nearest release.',
  },
  summary: {
    title: 'Window edge',
    accent: 'The 100 ms observation window is complete.',
  },
};

function timeToX(time: number) {
  const width = timeline.right - timeline.left;
  return timeline.left + (time / scenario.windowMs) * width;
}

function getReleaseY(taskId: TaskId) {
  return taskId === 'lidar' ? 74 : 114;
}

function getExecutionY(taskId: TaskId) {
  return taskId === 'lidar' ? 262 : 304;
}

function releaseClass(status: ReleaseStatus) {
  return `release ${status}`;
}

function formatTime(time: number) {
  return Number.isInteger(time) ? String(time) : time.toFixed(1);
}

function formatTaskList(taskIds: TaskId[]) {
  return taskIds.length > 0
    ? taskIds.map((taskId) => taskMap[taskId].label).join(', ')
    : 'none';
}

function getFinalExecution(steps: SimulationStep[]) {
  return steps[steps.length - 1]?.snapshot.execution ?? [];
}

function createVisualStep(
  steps: SimulationStep[],
  stepIndex: number,
  playbackTime: number,
): SimulationStep {
  const baseStep = steps[stepIndex];
  const finalExecution = getFinalExecution(steps);
  const snapshot = structuredClone(baseStep.snapshot);
  const activeBlock =
    baseStep.kind === 'start'
      ? finalExecution.find(
          (block) => playbackTime >= block.start && playbackTime < block.finish,
        )
      : undefined;

  snapshot.time = Number(playbackTime.toFixed(1));
  snapshot.currentBlockId = activeBlock?.id ?? null;

  for (const release of snapshot.releases) {
    if (release.status === 'future' && release.time <= playbackTime) {
      release.status = 'released';
    }
  }

  snapshot.execution = finalExecution
    .filter((block) => block.start <= playbackTime)
    .map((block) => {
      if (activeBlock?.id === block.id) {
        return {
          ...block,
          finish: playbackTime,
          response: null,
          status: 'running',
        };
      }

      return block.finish <= playbackTime ? block : null;
    })
    .filter((block): block is ExecutionBlock => block !== null);

  if (activeBlock) {
    snapshot.phase = 'start';
    return {
      kind: 'start',
      title: `Run ${activeBlock.label}`,
      body: `Progress: ${formatTime(playbackTime)} of ${activeBlock.finish} ms`,
      snapshot,
    };
  }

  return {
    ...baseStep,
    snapshot,
  };
}

function TimelineBoard({ step }: { step: SimulationStep }) {
  const state = step.snapshot;
  const currentX = timeToX(Math.min(state.time, scenario.windowMs));

  return (
    <section className="timeline-card" aria-label="Timeline board">
      <div className="card-title">
        <span>time board</span>
        <strong>jobs release / execution</strong>
      </div>
      <svg
        className="timeline-svg"
        viewBox={`0 0 ${timeline.viewBoxWidth} ${timeline.viewBoxHeight}`}
        role="img"
        aria-label="Superloop release and execution timeline"
      >
        <defs>
          <marker
            id="axisArrow"
            markerHeight="10"
            markerUnits="strokeWidth"
            markerWidth="10"
            orient="auto"
            refX="6"
            refY="3"
          >
            <path d="M0,0 L0,6 L7,3 z" fill="currentColor" />
          </marker>
        </defs>

        {Array.from({ length: 11 }, (_, index) => index * 10).map((time) => {
          const x = timeToX(time);
          return (
            <g key={time}>
              <line className="grid-line" x1={x} x2={x} y1={28} y2={392} />
              <text className="time-label top" x={x} y={172}>
                {time}
              </text>
              <text className="time-label bottom" x={x} y={350}>
                {time}
              </text>
            </g>
          );
        })}

        <text className="section-label" x="15" y="48">
          releases
        </text>
        <text className="section-label" x="15" y="226">
          execution
        </text>
        <text className="lane-label lidar-label" x="32" y="80">
          LiDAR
        </text>
        <text className="lane-label imu-label" x="32" y="120">
          IMU
        </text>
        <text className="lane-label lidar-label" x="32" y="268">
          LiDAR
        </text>
        <text className="lane-label imu-label" x="32" y="310">
          IMU
        </text>

        <line
          className="axis-line"
          markerEnd="url(#axisArrow)"
          x1={timeline.left - 6}
          x2={timeline.right + 30}
          y1={timeline.releaseAxisY}
          y2={timeline.releaseAxisY}
        />
        <line
          className="axis-line"
          markerEnd="url(#axisArrow)"
          x1={timeline.left - 6}
          x2={timeline.right + 30}
          y1={timeline.executionAxisY}
          y2={timeline.executionAxisY}
        />

        {state.releases.map((release) => (
          <ReleaseMarker key={release.id} release={release} />
        ))}

        {state.execution.map((block) => (
          <ExecutionBlockShape
            block={block}
            isActive={block.id === state.currentBlockId}
            key={block.id}
          />
        ))}

        <line
          className="current-line"
          x1={currentX}
          x2={currentX}
          y1={28}
          y2={392}
        />
        <circle
          className="current-dot"
          cx={currentX}
          cy={timeline.executionAxisY}
          r="7"
        />
        <text className="current-label" x={currentX + 9} y={31}>
          t={formatTime(state.time)}
        </text>
      </svg>
    </section>
  );
}

function ReleaseMarker({ release }: { release: Release }) {
  const x = timeToX(release.time);
  const y = getReleaseY(release.taskId);
  const color = taskColors[release.taskId];

  return (
    <g className={releaseClass(release.status)}>
      <line
        className="release-stem"
        style={{ color }}
        x1={x}
        x2={x}
        y1={y + 30}
        y2={y + 4}
      />
      <path
        className="release-arrow"
        d={`M ${x - 7} ${y + 5} L ${x} ${y - 10} L ${x + 7} ${y + 5}`}
      />
      <text className="release-text" x={x + 8} y={y + 5}>
        R{release.index}
      </text>
      {release.status === 'skipped' && (
        <g className="skip-cross">
          <line x1={x - 8} x2={x + 8} y1={y + 39} y2={y + 55} />
          <line x1={x + 8} x2={x - 8} y1={y + 39} y2={y + 55} />
        </g>
      )}
    </g>
  );
}

function ExecutionBlockShape({
  block,
  isActive,
}: {
  block: ExecutionBlock;
  isActive: boolean;
}) {
  const task = taskMap[block.taskId];
  const x = timeToX(block.start);
  const width = Math.max(12, timeToX(block.finish) - timeToX(block.start));
  const y = getExecutionY(block.taskId) - 26;
  const status = isActive ? 'running' : block.status;
  const labelInside = width >= 84;

  return (
    <g className={`execution-block ${status}`}>
      <rect height="34" rx="6" width={width} x={x} y={y} />
      <text
        className={labelInside ? 'inside' : 'outside'}
        x={labelInside ? x + width / 2 : x + width + 10}
        y={y + 22}
      >
        {block.label}
      </text>
      {block.status === 'miss' && (
        <>
          <line
            className="deadline-line"
            x1={timeToX(block.releaseTime + task.d)}
            x2={timeToX(block.releaseTime + task.d)}
            y1={y - 12}
            y2={y + 46}
          />
          <text
            className="deadline-text"
            x={timeToX(block.releaseTime + task.d) + 7}
            y={y - 16}
          >
            D
          </text>
        </>
      )}
    </g>
  );
}

function LoopMachine({ step }: { step: SimulationStep }) {
  const phase = step.snapshot.phase;
  const activeIndex = Math.max(
    0,
    phaseOrder.findIndex((item) => item.id === phase),
  );
  const activePhase = phaseOrder[activeIndex];

  return (
    <section className="loop-card" aria-label="Superloop pass">
      <div className="card-title">
        <span>superloop</span>
        <strong>pass #{step.snapshot.loopPass}</strong>
      </div>
      <div className="loop-orbit" aria-hidden="true">
        {phaseOrder.map((item, index) => (
          <div
            className={`loop-node ${phase === item.id ? 'active' : ''}`}
            key={item.id}
            data-phase={item.id}
            style={{ '--i': index } as React.CSSProperties}
          >
            <strong>{item.label}</strong>
            <span>{item.hint}</span>
          </div>
        ))}
        <div className="loop-core">
          <span>while(1)</span>
          <strong>{activePhase.core}</strong>
        </div>
      </div>
      <SystemNote step={step} />
    </section>
  );
}

function SystemNote({ step }: { step: SimulationStep }) {
  const state = step.snapshot;
  const note = phaseNotes[state.phase];
  const readyTasks = scenario.order.filter(
    (taskId) => state.time >= state.nextRelease[taskId],
  );
  const nearestRelease = Math.min(
    ...scenario.tasks.map((task) => state.nextRelease[task.id]),
  );
  const lastResponse = state.lastResponse;

  return (
    <aside className="system-note" aria-label="System explanation">
      <div className="system-note__head">
        <span>model note</span>
        <strong>{note.title}</strong>
      </div>

      <div className="note-grid">
        <div>
          <span>ready now</span>
          <strong>{formatTaskList(readyTasks)}</strong>
        </div>
        <div>
          <span>nearest release</span>
          <strong>{nearestRelease} ms</strong>
        </div>
      </div>

      <div className="pointer-table" aria-label="Task next releases">
        {scenario.tasks.map((task) => {
          const stats = state.stats[task.id];

          return (
            <div className="pointer-row" key={task.id}>
              <strong>{task.label}</strong>
              <span>next {state.nextRelease[task.id]} ms</span>
              <span>{stats.runs} run</span>
              <span>{stats.skips} skip</span>
            </div>
          );
        })}
      </div>

      <p>{note.accent}</p>

      {lastResponse && (
        <div className="response-chip">
          <span>{lastResponse.label}</span>
          <strong>
            R={lastResponse.response} / D={lastResponse.deadline}
          </strong>
        </div>
      )}
    </aside>
  );
}

function StatusDock({
  step,
  index,
  total,
}: {
  step: SimulationStep;
  index: number;
  total: number;
}) {
  const state = step.snapshot;
  const lidar = state.stats.lidar;
  const imu = state.stats.imu;

  return (
    <section className="status-dock" aria-label="Current system configuration">
      <div className="event-tile">
        <span>event</span>
        <strong>{step.title}</strong>
        <code>{step.body}</code>
      </div>
      <MetricTile label="t_cur" unit="ms" value={formatTime(state.time)} />
      <MetricTile label="step" value={`${index + 1}/${total}`} />
      <TaskTile
        color="lidar"
        misses={lidar.misses}
        next={state.nextRelease.lidar}
        runs={lidar.runs}
        skips={lidar.skips}
        title="LiDAR"
      />
      <TaskTile
        color="imu"
        misses={imu.misses}
        next={state.nextRelease.imu}
        runs={imu.runs}
        skips={imu.skips}
        title="IMU"
      />
      <div className="response-tile">
        <span>response</span>
        <strong>
          {state.lastResponse
            ? `${state.lastResponse.response} / ${state.lastResponse.deadline} ms`
            : '-'}
        </strong>
        <small>
          {state.lastResponse?.missed
            ? 'MISS'
            : state.lastResponse
              ? 'OK'
              : 'waiting'}
        </small>
      </div>
    </section>
  );
}

function MetricTile({
  label,
  unit,
  value,
}: {
  label: string;
  unit?: string;
  value: string;
}) {
  return (
    <div className="metric-tile">
      <span>{label}</span>
      <strong>
        {value}
        {unit && <small>{unit}</small>}
      </strong>
    </div>
  );
}

function TaskTile({
  color,
  misses,
  next,
  runs,
  skips,
  title,
}: {
  color: TaskId;
  misses: number;
  next: number;
  runs: number;
  skips: number;
  title: string;
}) {
  return (
    <div className={`task-tile ${color}`}>
      <span>{title}</span>
      <strong>next {next}</strong>
      <div className="task-counters">
        <span>
          <b>{runs}</b>
          run
        </span>
        <span>
          <b>{misses}</b>
          miss
        </span>
        <span>
          <b>{skips}</b>
          skip
        </span>
      </div>
    </div>
  );
}

function Controls({
  current,
  isPlaying,
  onNext,
  onPlay,
  onPrev,
  onReset,
  onScrub,
  total,
}: {
  current: number;
  isPlaying: boolean;
  onNext: () => void;
  onPlay: () => void;
  onPrev: () => void;
  onReset: () => void;
  onScrub: (value: number) => void;
  total: number;
}) {
  return (
    <section className="controls-band">
      <div className="button-row">
        <button onClick={onReset} type="button">
          Reset
        </button>
        <button disabled={current === 0} onClick={onPrev} type="button">
          Prev
        </button>
        <button className="primary" onClick={onPlay} type="button">
          {isPlaying ? 'Pause' : 'Play'}
        </button>
        <button disabled={current === total - 1} onClick={onNext} type="button">
          Next
        </button>
      </div>
      <input
        aria-label="Step"
        max={total - 1}
        min={0}
        onChange={(event) => onScrub(Number(event.target.value))}
        step={1}
        type="range"
        value={current}
      />
    </section>
  );
}

function App() {
  const steps = useMemo(() => simulateSuperloop(), []);
  const [currentStep, setCurrentStep] = useState(0);
  const [playbackTime, setPlaybackTime] = useState(steps[0].snapshot.time);
  const [isPlaying, setIsPlaying] = useState(false);
  const animationRef = useRef<number | null>(null);
  const animationStartRef = useRef(0);
  const playStartTimeRef = useRef(0);

  const step = useMemo(
    () => createVisualStep(steps, currentStep, playbackTime),
    [currentStep, playbackTime, steps],
  );

  useEffect(() => {
    if (!isPlaying) {
      return;
    }

    if (currentStep >= steps.length - 1) {
      setIsPlaying(false);
      return;
    }

    const targetTime = steps[currentStep + 1].snapshot.time;
    const sourceTime = playbackTime;
    const durationMs =
      targetTime > sourceTime
        ? Math.max(420, ((targetTime - sourceTime) / SIM_MS_PER_SECOND) * 1000)
        : 520;

    animationStartRef.current = performance.now();
    playStartTimeRef.current = sourceTime;

    const tick = (now: number) => {
      const progress = Math.min(
        1,
        (now - animationStartRef.current) / durationMs,
      );
      const easedProgress = 1 - Math.pow(1 - progress, 3);
      const nextTime = sourceTime + (targetTime - sourceTime) * easedProgress;

      setPlaybackTime(Number(nextTime.toFixed(2)));

      if (progress >= 1) {
        setPlaybackTime(targetTime);
        setCurrentStep((stepIndex) =>
          Math.min(steps.length - 1, stepIndex + 1),
        );
        animationRef.current = null;
        return;
      }

      animationRef.current = window.requestAnimationFrame(tick);
    };

    animationRef.current = window.requestAnimationFrame(tick);

    return () => {
      if (animationRef.current !== null) {
        window.cancelAnimationFrame(animationRef.current);
        animationRef.current = null;
      }
    };
  }, [currentStep, isPlaying, playbackTime, steps]);

  const goToStep = (value: number) => {
    const nextStep = Math.max(0, Math.min(steps.length - 1, value));
    setIsPlaying(false);
    setCurrentStep(nextStep);
    setPlaybackTime(steps[nextStep].snapshot.time);
  };

  return (
    <main className="app-shell">
      <header className="top-hud">
        <div>
          <p>STM32 non-preemptive scheduler</p>
          <h1>Superloop visual lab</h1>
        </div>
        <div className="param-strip" aria-label="Scenario parameters">
          <span>{'order: LiDAR -> IMU'}</span>
          <span>LiDAR C=20 T=D=50</span>
          <span>IMU C=10 T=D=20</span>
          <span>window 100 ms</span>
        </div>
      </header>

      <section className="stage-grid">
        <TimelineBoard step={step} />
        <LoopMachine step={step} />
      </section>

      <StatusDock index={currentStep} step={step} total={steps.length} />

      <Controls
        current={currentStep}
        isPlaying={isPlaying}
        onNext={() => goToStep(currentStep + 1)}
        onPlay={() => setIsPlaying((value) => !value)}
        onPrev={() => goToStep(currentStep - 1)}
        onReset={() => goToStep(0)}
        onScrub={goToStep}
        total={steps.length}
      />
    </main>
  );
}

export default App;
