export class StripChart {
  constructor(canvas, series, windowMs = 12000) {
    this.canvas = canvas;
    this.ctx = canvas.getContext("2d");
    this.series = series;
    this.windowMs = windowMs;
    this.samples = [];
    this.pixelRatio = window.devicePixelRatio || 1;
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(canvas);
    this.resize();
  }

  push(timestampMs, values) {
    this.samples.push({ timestampMs, values });
    const cutoff = timestampMs - this.windowMs;
    while (this.samples.length > 0 && this.samples[0].timestampMs < cutoff) {
      this.samples.shift();
    }
  }

  clear() {
    this.samples = [];
    this.draw();
  }

  resize() {
    const rect = this.canvas.getBoundingClientRect();
    this.canvas.width = Math.max(1, Math.floor(rect.width * this.pixelRatio));
    this.canvas.height = Math.max(1, Math.floor(rect.height * this.pixelRatio));
    this.draw();
  }

  draw() {
    const ctx = this.ctx;
    const width = this.canvas.width;
    const height = this.canvas.height;
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = "#101417";
    ctx.fillRect(0, 0, width, height);

    const pad = 26 * this.pixelRatio;
    const plotWidth = Math.max(1, width - (pad * 2));
    const plotHeight = Math.max(1, height - (pad * 2));

    ctx.strokeStyle = "rgba(210, 220, 220, 0.11)";
    ctx.lineWidth = 1 * this.pixelRatio;
    for (let i = 0; i <= 4; i += 1) {
      const y = pad + (plotHeight * i / 4);
      ctx.beginPath();
      ctx.moveTo(pad, y);
      ctx.lineTo(width - pad, y);
      ctx.stroke();
    }

    if (this.samples.length < 2) {
      return;
    }

    const firstTs = this.samples[0].timestampMs;
    const lastTs = this.samples[this.samples.length - 1].timestampMs;
    const timeSpan = Math.max(1, lastTs - firstTs);
    const allValues = [];
    this.samples.forEach((sample) => {
      this.series.forEach((item) => allValues.push(sample.values[item.key] ?? 0));
    });
    const minValue = Math.min(...allValues, -1);
    const maxValue = Math.max(...allValues, 1);
    const valueSpan = Math.max(1, maxValue - minValue);

    this.series.forEach((item) => {
      ctx.strokeStyle = item.color;
      ctx.lineWidth = 2 * this.pixelRatio;
      ctx.beginPath();
      this.samples.forEach((sample, index) => {
        const x = pad + (((sample.timestampMs - firstTs) / timeSpan) * plotWidth);
        const y = pad + ((maxValue - (sample.values[item.key] ?? 0)) / valueSpan) * plotHeight;
        if (index === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      });
      ctx.stroke();
    });
  }
}
