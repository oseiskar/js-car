/* globals d3 */
function buildCarRenderer(car, trackCoords, color) {
  const carCoords = trackCoords.append('g');

  const carBody = carCoords
      .append('rect')
        .attr('x', -car.properties.width*0.5)
        .attr('y', -car.properties.length*0.5)
        .attr('width', car.properties.width)
        .attr('height', car.properties.length)
        .attr('fill', color);

  const wheels = [[true,-1], [true,1], [false,-1], [false,1]].map(([isFront, side]) => {
    const coords = carCoords.append('g');
    const wheelW = car.properties.width * 0.2;
    const wheelL = car.properties.width * 0.4;
    const rect = coords.append('rect')
      .attr('x', -wheelW*0.5)
      .attr('y', -wheelL*0.5)
      .attr('width', wheelW)
      .attr('height', wheelL);

    return {
      coords,
      rect,
      isFront,
      side,
      x: car.properties.width * 0.5 * side,
      y: car.properties.length * 0.5 * (isFront ? -1 : 1)
    };
  });

  return () => {
    const [rotLeft, rotRight] = car.frontWheelAngles;

    // car coordinate system
    carCoords.attr('transform', `
      translate(${car.pos[0]}, ${car.pos[1]})
      rotate(${car.rot/Math.PI*180.0 + 90})`);

    wheels.forEach(wheel => {
      const rot = wheel.isFront ? (wheel.side > 0 ? rotLeft : rotRight) : 0.0;
      wheel.coords.attr('transform', `
        translate(${wheel.x}, ${wheel.y})
        rotate(${rot/Math.PI*180.0})`);

      const slip = wheel.isFront ? car.slip.front : car.slip.back;
      const color = slip ? 'gray' : 'black';
      wheel.rect.attr('fill', color);
    });
  };
}

function buildAiVisualizer(coords, visualizationGenerator, scale) {
  const group = coords.append('g');
  function initPlanVisualizer(data) {
    function velocityColormap(x) {
      const maxVel = 10;
      const minVel = 6.0;
      const rel = Math.max(0.0, Math.min((x-minVel) / (maxVel - minVel), 1.0));
      return `rgb(${(1.0-rel)*255}, ${rel*255}, 0)`;
    }

    return (data) => {
      group.selectAll('circle').remove();
      group.selectAll('circle')
        .data(data.route)
        .enter()
        .append('circle')
        .attr('cx', (d, idx) => d[0])
        .attr('cy', (d, idx) => d[1])
        .attr('r', 2.0 / scale)
        .attr('fill-opacity', 0.5)
        .attr('fill', (_, idx) => velocityColormap(data.velocities[idx]));
    };
  }

  function initRlVisualizer(data) {
    function stride(n, includeEnd = true) {
      let i = 0;
      return (array) => array.filter(() => {
        if (includeEnd && i === array.length - 1) return true;
        return (i++ % n) === 0;
      });
    }

    function rewardColormap(x) {
      const minReward = -5;
      const maxReward = 2;
      const rel = Math.max(0.0, Math.min((x-minReward) / (maxReward - minReward), 1.0));
      return `rgb(${(1.0-rel)*255}, ${rel*255}, 0)`;
    }

    return (data) => {
      group.selectAll('g').remove();
      group.selectAll('g')
        .data(data.traces)
        .enter()
        .append('g')
        .selectAll('circle')
        .data(d => stride(4)(d))
        .enter()
        .append('circle')
        .attr('cx', d => d[0])
        .attr('cy', d => d[1])
        .attr('r', 2.0 / scale)
        .attr('fill-opacity', 0.25)
        .attr('fill', d => rewardColormap(d[2]));
    };
  }

  function init(data) {
    if (data.route) return initPlanVisualizer(data);
    if (data.traces) return initRlVisualizer(data);
    return null;
  }

  let renderedVersion;
  let visualizer;

  return () => {
    const data = visualizationGenerator();
    if (!data || data.version === renderedVersion) return;
    renderedVersion = data.version;

    if (!visualizer) visualizer = init(data);
    if (!visualizer) return;

    visualizer(data);
  };
}

function buildTrackRenderer(track, svg, width, height) {
  const metricSize = track.size;
  const scale = Math.min(width / metricSize.width, height / metricSize.height);
  const [szX, szY] = [width, height];

  const trackCoords = svg.append('g').attr('transform', `translate(0, ${height})`)
    .append('g').attr('transform', 'scale(1,-1)')
    .append('g').attr('transform', `translate(${width/2}, ${height/2})`)
    .append('g').attr('transform', `scale(${scale})`);

  // boundary
  trackCoords.append('rect')
    .attr('x', -metricSize.width/2)
    .attr('y', -metricSize.height/2)
    .attr('width', metricSize.width)
    .attr('height', metricSize.height)
    .attr('stroke', 'black').attr('stroke-width', 1 / scale)
    //.attr('fill', '#f0f0f0');
    .attr('fill', 'none');

  // track
  const trackPoints = track.points; // close loop
  const trackLine = d3.line()
    .x(d => d[0]).y(d => d[1])
    .curve(d3.curveLinearClosed)(trackPoints);
  trackCoords.append('path')
    .attr('d', trackLine)
    .attr('stroke', '#dddddd')
    .attr('stroke-width', track.trackWidth) // meters
    .attr('fill', 'none');

  trackCoords.append('path')
    .attr('d', trackLine)
    .attr('stroke', '#ffff00')
    .attr('stroke-width', 2.0 / scale)
    .attr('stroke-dasharray', `0.2 0.3`)
    .attr('fill', 'none');

  const aiVisus = trackCoords.append('g');

  const carColors = ['green', 'red', 'blue'];
  let carIndex = 0;

  function htmlEscape(str) {
    // why's this not a standard part of ECMAScript-something in 2019 ?!!!
    // seriously weak, guys...
    const dummy = document.createElement('div');
    dummy.innerText = str;
    return dummy.innerHTML;
  }

  const carRenderers = track.cars.map(car => {
    const color = carColors[carIndex++ % carColors.length];
    svg.append('text')
      .attr('x', 20)
      .attr('y', carIndex * 20 + 10)
      .attr('fill', color)
      .html('&bull; ' + htmlEscape(car.name));

    const visu = car.steering.visualization;
    const renderAiPlan = visu ?
      buildAiVisualizer(aiVisus, visu, scale) :
      () => {};

    const renderCar = buildCarRenderer(car.model, trackCoords, color);
    return () => {
      renderAiPlan();
      renderCar();
    };
  });

  return () => {
    carRenderers.forEach(f => f());
  };
}
