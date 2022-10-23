package s0579030;

import java.awt.Color;
import java.awt.Graphics2D;
import java.awt.Point;
import java.awt.geom.Line2D;
import java.awt.geom.Path2D;
import java.awt.geom.PathIterator;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Collections;
import java.util.PriorityQueue;

import lenz.htw.ai4g.ai.AI;
import lenz.htw.ai4g.ai.DivingAction;
import lenz.htw.ai4g.ai.Info;
import lenz.htw.ai4g.ai.PlayerAction;

public class Rakete extends AI {
	Point[] pearls = info.getScene().getPearl();
	float screenRatio = (float) (info.getScene().getWidth() * 1.0 / info.getScene().getHeight());
	int widthDivision = 150;
	int heightDivision = (int) (widthDivision * (1 / screenRatio));
	Path2D[] obstacles = info.getScene().getObstacles();
	Vertex[] vertices = new Vertex[widthDivision * heightDivision];
	int pointsVisited = 0;
	float errorMargin = 1.1f;
	ArrayList<Vector2D> idealPath = new ArrayList<>();
	ArrayList<ArrayList<Vector2D>> allPathsFromSurface = new ArrayList<>();
	ArrayList<ArrayList<Vector2D>> allPathsToNextPearl = new ArrayList<>();
	int[] pathsFromSurfaceAirUsages = new int[pearls.length];
	int[] pathsToNextPearlAirUsages = new int[pearls.length -1];

	public Rakete(Info info) {
		super(info);
		enlistForTournament(579030, 577618);

		// Create path between pearls to follow
		sortPearls();
		setGrid();
		setNeighbours();
		setSurfacePaths();
		setNextPearlPaths();
		setIdealPath();
	}

	// Sort pearls by x
	public void sortPearls() {
		// Sort pearls by x value with bubble sort
		for(int i = 0; i < pearls.length - 1; i++) {
			for(int j = 0; j < pearls.length - i - 1; j++) {
				if(pearls[j].getX() > pearls[j + 1].getX()) {
					Point temp = pearls[j];
					pearls[j] = pearls[j + 1];
					pearls[j + 1] = temp;
				}
			}
		}
	}

	// Set graph grid
	public void setGrid() {
		Vector2D pearlPosition = new Vector2D((float) pearls[0].getX(), (float) pearls[0].getY());

		// Scan grid for obstacles and set free tiles to true in freeSpace
		for (int x = 0; x < widthDivision; x++) {
			for (int y = 0; y < heightDivision; y++) {
				Rectangle2D currentTile = new Rectangle2D.Float();
				currentTile.setFrame(x * info.getScene().getWidth() / widthDivision, y * info.getScene().getHeight() / heightDivision, info.getScene().getWidth() / widthDivision, info.getScene().getHeight() / heightDivision);

				// Check each obstacle if it intersects with current tile
				for (Path2D obstacle : obstacles) {
					if (obstacle.intersects(currentTile)) {
						vertices[x + y * widthDivision] = null;
						break;
					} else {
						// If tile is free, create new vertex for graph in the middle of the tile
						Vector2D vertexPosition = new Vector2D((float) currentTile.getCenterX(), (float) currentTile.getCenterY());
						vertices[x + y * widthDivision] = new Vertex(vertexPosition, pearlPosition);
					}
				}
			}
		}
	}

	// Set graph vertex neighbours
	public void setNeighbours() {
		// Set the neighbours for each vertex
		for(int vertex = 0; vertex < vertices.length; vertex++) {
			Vertex leftNeighbour = vertices[vertex];
			Vertex rightNeighbour = vertices[vertex];
			Vertex upperNeighbour = vertices[vertex];
			Vertex lowerNeighbour = vertices[vertex];

			// Only check for the vertices in free space
			if(vertices[vertex] != null) {
				// Left
				if(vertex % widthDivision == 0) {
					leftNeighbour = null;
				}
				// Right
				if(vertex % widthDivision == widthDivision - 1) {
					rightNeighbour = null;

				}
				// Up
				if(vertex < widthDivision) {
					upperNeighbour = null;

				}
				// Down
				if(vertex > vertices.length - widthDivision - 1) {
					lowerNeighbour = null;
				}

				if(leftNeighbour == vertices[vertex]) {
					leftNeighbour = vertices[vertex - 1];
				}
				if(rightNeighbour == vertices[vertex]) {
					rightNeighbour = vertices[vertex + 1];
				}
				if(upperNeighbour == vertices[vertex]) {
					upperNeighbour = vertices[vertex - widthDivision];
				}
				if(lowerNeighbour == vertices[vertex]) {
					lowerNeighbour = vertices[vertex + widthDivision];
				}

				vertices[vertex].setNeighbour(0, leftNeighbour); // Left
				vertices[vertex].setNeighbour(1, rightNeighbour); // Right
				vertices[vertex].setNeighbour(2, upperNeighbour); // Up
				vertices[vertex].setNeighbour(3, lowerNeighbour); // Down
			}
		}
	}

	// Set path from surface to pearl
	public void setSurfacePaths() {
		for(int i = 0; i < pearls.length; i++) {
			Vector2D pointAbovePearl = new Vector2D((float) pearls[i].getX(), 0);
			Vector2D pearlPosition = new Vector2D((float) pearls[i].getX(), (float) pearls[i].getY());

			// Find the shortest path from surface to current pearl and smooth it
			ArrayList<Vector2D> pathFromSurface = aStarPathFinding(pointAbovePearl, pearlPosition);
			pathFromSurface = smoothPath(pathFromSurface);
			pathFromSurface.add(pearlPosition);

			pathsFromSurfaceAirUsages[i] = calculateAirUsage(pathFromSurface, errorMargin);
			allPathsFromSurface.add(i, pathFromSurface);
		}
	}

	// Set path to succeeding pearl
	public void setNextPearlPaths() {
		for(int i = 0; i < pearls.length - 1; i++) {
			Vector2D pearlPosition = new Vector2D((float) pearls[i].getX(), (float) pearls[i].getY());
			Vector2D nextPearlPosition = new Vector2D((float) pearls[i + 1].getX(), (float) pearls[i + 1].getY());

			// Find the shortest path to next pearl and smooth it
			ArrayList<Vector2D> pathToNextPearl = aStarPathFinding(pearlPosition, nextPearlPosition);
			pathToNextPearl = smoothPath(pathToNextPearl);
			pathToNextPearl.add(nextPearlPosition);

			pathsToNextPearlAirUsages[i] = calculateAirUsage(pathToNextPearl, errorMargin);
			allPathsToNextPearl.add(i, pathToNextPearl);
		}
	}

	// Set path between all pearls
	public void setIdealPath() {
		// Find ideal path with partial paths between pearls and paths to surface
		// Create partial paths between reachable pearls and add to final path
		for(int i = 0; i < pearls.length - 1;) {
			// Set 1 because at least one pearl is in current partial path
			int pearlsInPartialPath = 1;

			// Check how many pearls are available from current pearl
			ArrayList<Vector2D> partialPathBetweenPearls = new ArrayList<>();
			boolean partialPathDone = false;

			if(i < pearls.length - 1) {
				idealPath.addAll(allPathsFromSurface.get(i));
				partialPathBetweenPearls.addAll(allPathsFromSurface.get(i));
				partialPathBetweenPearls.addAll(allPathsToNextPearl.get(i));

				for (int j = i; j < pearls.length - 2 && !partialPathDone; j++) {
					if (info.getMaxAir() > calculateAirUsage(partialPathBetweenPearls, errorMargin) + pathsFromSurfaceAirUsages[j + 1]) {
						// Add next pearl to partial path
						idealPath.addAll(allPathsToNextPearl.get(j));
						partialPathBetweenPearls.addAll(allPathsToNextPearl.get(j + 1));
						pearlsInPartialPath += 1;
					} else {
						// Add path to surface to partial path
						Collections.reverse(allPathsFromSurface.get(j));
						idealPath.addAll(allPathsFromSurface.get(j));
						partialPathDone = true;
					}
				}
			}

			// Continue from last pearl in final path
			i += pearlsInPartialPath;
		}

		idealPath.addAll(allPathsToNextPearl.get(pearls.length - 2));
	}

	// Skip all vertices in path that are unnecessary
	public ArrayList<Vector2D> smoothPath(ArrayList<Vector2D> path) {
		// Add the first vertex of the segment as a starting position
		ArrayList<Vector2D> smoothPath = new ArrayList<>();
		smoothPath.add(path.get(0));

		// Check each vertex in segment if a line to it would intersect with the obstacles
		for(int i = 1; i < path.size(); i++) {
			// Creating a line between the last good path vertex and the current segment vertex
			Line2D lineBetweenVertices = new Line2D.Float();
			lineBetweenVertices.setLine(smoothPath.get(smoothPath.size()-1).getX(), smoothPath.get(smoothPath.size()-1).getY(), path.get(i).getX(), path.get(i).getY());

			// Check each obstacle if it intersects with the line
			for(Path2D obstacle : obstacles) {
				if(intersects(lineBetweenVertices, obstacle)) {
					// If they intersect, add the previous vertex to the smooth path and check a new line
					smoothPath.add(path.get(i-1));
					break;
				}
			}
		}

		return smoothPath;
	}

	// Calculate air necessary to swim provided path
	public int calculateAirUsage(ArrayList<Vector2D> path, float errorMargin) {
		int pathLength = 0;
		int airUsageForPath;
		for(int i = 0; i < path.size() - 1; i++) {
			pathLength += path.get(i+1).subtractVector(path.get(i)).getLength();
		}
		airUsageForPath = pathLength / (int) Math.ceil(info.getMaxAcceleration());

		return (int) (airUsageForPath * errorMargin);
	}

	// Check if a line intersects with an obstacle
	public boolean intersects(Line2D line, Path2D path) {
		Point2D start = null;
		Point2D point1 = null;
		Point2D point2 = null;
		for (PathIterator pi = path.getPathIterator(null); !pi.isDone(); pi.next()) {
			float[] coordinates = new float[6];
			switch (pi.currentSegment(coordinates)) {
				case PathIterator.SEG_MOVETO -> {
					point2 = new Point2D.Float(coordinates[0], coordinates[1]);
					point1 = null;
					start = (Point2D) point2.clone();
				}
				case PathIterator.SEG_LINETO -> {
					point1 = point2;
					point2 = new Point2D.Float(coordinates[0], coordinates[1]);
				}
				case PathIterator.SEG_CLOSE -> {
					point1 = point2;
					point2 = start;
				}
			}
			if (point1 != null) {
				Line2D segment = new Line2D.Float(point1, point2);
				if (segment.intersectsLine(line))
					return true;
			}
		}
		return false;
	}

//	@Override
//	// Draw path
//	public void drawDebugStuff(Graphics2D gfx) {
//		for(int i = 0; i < idealPath.size()-1; i++) {
//			gfx.setColor(Color.red);
//			gfx.drawLine((int) idealPath.get(i).getX(), (int) idealPath.get(i).getY(), (int) idealPath.get(i+1).getX(), (int) idealPath.get(i+1).getY());
//		}
//	}

	// Returns a path from the closest vertex to the starting position to the closest vertex to the next pearl position
	public ArrayList<Vector2D> aStarPathFinding(Vector2D startingPosition, Vector2D pearlPosition) {
		// Set all vertices of the graph to infinite distance with no previous node
		for(Vertex vertex: vertices) {
			if(vertex != null) {
				vertex.setDistanceToEnd(pearlPosition.subtractVector(startingPosition).getLength());
				vertex.setDistanceFromStartPosition(Double.POSITIVE_INFINITY);
				vertex.setPreviousVertex(null);
				vertex.setExplored(false);
			}
		}

		// Add a queue for the next node with the smallest distance
		Vertex currentVertex;
		PriorityQueue<Vertex> unexploredVertices = new PriorityQueue<>();

		// Find the closest vertex to start position and to pearl position
		double startToVertexDistance = Double.POSITIVE_INFINITY;
		Vertex closestVertexToStart = vertices[0];

		double pearlToVertexDistance = Double.POSITIVE_INFINITY;
		Vertex closestVertexToPearl = vertices[0];

		// Check distance to start and pearl for each vertex
		for (Vertex vertexToMeasure : vertices) {
			// Only check existing vertices
			if (vertexToMeasure != null) {
				double startDistanceToCurrentVertex = vertexToMeasure.getLocation().subtractVector(startingPosition).getLength();
				double pearlDistanceToCurrentVertex = vertexToMeasure.getLocation().subtractVector(pearlPosition).getLength();

				if (startDistanceToCurrentVertex < startToVertexDistance) {
					startToVertexDistance = startDistanceToCurrentVertex;
					closestVertexToStart = vertexToMeasure;
				}

				if (pearlDistanceToCurrentVertex < pearlToVertexDistance) {
					pearlToVertexDistance = pearlDistanceToCurrentVertex;
					closestVertexToPearl = vertexToMeasure;
				}
			}
		}

		// Add start node to the priority queue and set destination node
		unexploredVertices.add(closestVertexToStart);
		closestVertexToStart.setDistanceFromStartPosition(closestVertexToStart.getDistanceToEnd());
		closestVertexToStart.setPreviousVertex(null);
		Vertex destination = closestVertexToPearl;

		// For every neighbour of the current node update the distance
		// Set new previous node to current node
		while(!destination.getExplored()) {
			// Pull the nearest element out of the queue and get its neighbours
			currentVertex = unexploredVertices.poll();
			Vertex[] neighbours;

			if (currentVertex != null) {
				neighbours = currentVertex.getNeighbours();
			}
			else {
				return null;
			}

			// Look at all neighbours and check/update their distances
			for(Vertex neighbour : neighbours) {
				if(neighbour != null) {
					if (!neighbour.getExplored()) {
						// If the neighbour doesn't have a distance yet, set it and queue it
						if (neighbour.getDistanceFromStartPosition() == Double.POSITIVE_INFINITY) {
							neighbour.setDistanceFromStartPosition(currentVertex.getDistanceFromStartPosition() + 1 + currentVertex.getDistanceToEnd());
							neighbour.setPreviousVertex(currentVertex);
							unexploredVertices.add(neighbour);
						}
						// If it has a distance, just update it
						else {
							neighbour.setDistanceFromStartPosition(currentVertex.getDistanceFromStartPosition() + 1 + currentVertex.getDistanceToEnd());
							neighbour.setPreviousVertex(currentVertex);
						}
					}
				}
			}
			// Set current node to explored, so it won't be checked again
			currentVertex.setExplored(true);
		}

		// Backtrack the path from the destination to the start and return it as string
		currentVertex = destination;
		ArrayList<Vector2D> path = new ArrayList<>();

		while(currentVertex != null) {
			path.add(currentVertex.getLocation());
			currentVertex = currentVertex.getPreviousVertex();
		}

		// Make path from start to pearl
		Collections.reverse(path);

		return path;
	}

	@Override
	public String getName() {
		return "Rakete";
	}

	@Override
	public Color getPrimaryColor() {
		return Color.CYAN;
	}

	@Override
	public Color getSecondaryColor() {
		return Color.BLUE;
	}

	@Override
	public PlayerAction update() {
		// Get diver position
		double startX = info.getX();
		double startY = info.getY();
		Vector2D startVector = new Vector2D((float) startX, (float) startY);

		// Get next point in path ArrayList
		double seekX = idealPath.get(pointsVisited).getX();
		double seekY = idealPath.get(pointsVisited).getY();
		Vector2D seekVector = new Vector2D((float) seekX, (float) seekY);

		// Check if point on path was visited
		if(Math.abs(startVector.getX() - seekVector.getX()) < 1 && Math.abs(startVector.getY() - seekVector.getY()) < 1 && pointsVisited < idealPath.size() - 1) {
			pointsVisited++;
		}

		// Seek pearl
		Vector2D seekDirection = seekVector.subtractVector(startVector);
		seekDirection = seekDirection.normalize();

		// Calculate direction radiant value
		float direction = (float) Math.atan2(seekDirection.getY(), seekDirection.getX());
		return new DivingAction(1, -direction);
	}
}
