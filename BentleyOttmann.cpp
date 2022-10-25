#include "Common.h"

#include "BentleyOttmann.h"
#include "Algorithm.h"

// Create an instance of the BentleyOttmann and initialize it.
BentleyOttmann* BentleyOttmann::CreateBentleyOttmann(const Vector2* polygon, int count)
{	
	BentleyOttmann* instance = new BentleyOttmann(polygon, count);
	if (instance && instance->Initialize())
		return instance;

	return NULL;
}

// Initialize BentleyOttmann.
bool BentleyOttmann::Initialize()
{
	int* sortedVertexIndex = new int[mCount];
	int* vertexOrder = new int[mCount];
	
	// Sort the vertices by x.
	for (int i = 0; i < mCount; ++i)
		sortedVertexIndex[i] = i;
	QuickSort(0, mCount - 1, sortedVertexIndex);

	// Get the order of the vertex in the sorted list.
	for (int i = 0; i < mCount; ++i)
	{
		vertexOrder[sortedVertexIndex[i]] = i;
	}
	
	// Create segments list.
	mSegments = new Segment[mCount];
	int* startIndex = new int[mCount];
	for (int i = 0; i < mCount; ++i)
	{		
		int i0 = i;
		int i1 = (i + 1) % mCount;

		const Vector2& p0 = mVertices[i0];
		const Vector2& p1 = mVertices[i1];

		// Get the left vertex and the right vertex.
		if (vertexOrder[i0] < vertexOrder[i1])
		{
			mSegments[i].leftVertex = p0;
			mSegments[i].rightVertex = p1;

			startIndex[i] = i0;
		}
		else
		{
			mSegments[i].leftVertex = p1;
			mSegments[i].rightVertex = p0;

			startIndex[i] = i1;
		}

		// Get the delta vector.
		mSegments[i].delta = mSegments[i].rightVertex - mSegments[i].leftVertex;
	}
	
	// Create event list.
	mEventList = new Event[mCount];
	int* segmentOrder = new int[mCount];
	for (int i = 0; i < mCount; ++i)
		segmentOrder[i] = -1;
	for (int i = 0; i < mCount; ++i)
	{
		Event& event = mEventList[i];

		// Get the vertex of this event.
		int vertexIndex = sortedVertexIndex[i];
		event.vertex = mVertices[vertexIndex];

		// Create segment list of this event.
		int i0 = (vertexIndex - 1 + mCount) % mCount;
		int i1 = vertexIndex;
		if (vertexIndex == startIndex[i0] && vertexIndex == startIndex[i1])
		{
			// Two edges starts with the vertex.
			event.eventType = Insert;
			
			// Sort the 2 segments.
			const Vector2& d0 = mSegments[i0].delta;
			const Vector2& d1 = mSegments[i1].delta;
			if (d0.y * d1.x > d0.x * d1.y)
			{
				// Swap the index.
				int t = i0;
				i0 = i1;
				i1 = t;
			}

			// Push the two edges into the edge list.
			if (i == 0)
			{
				event.operationPosition = 0;
				event.segmentIndexList.push_back(i0);
				event.segmentIndexList.push_back(i1);

				segmentOrder[i0] = 0;
				segmentOrder[i1] = 1;
			}
			else
			{
				// Find the insert position by searching the edge list of the previous event.
				const Array<int>& preList = mEventList[i-1].segmentIndexList;
				int insertIndex = 0;				
				for (; insertIndex < (int)preList.size(); ++insertIndex)
				{
					const Segment& segment = mSegments[preList[insertIndex]];
					if (Segment::IsPointBelowSegment(event.vertex, segment))
						break;
				}
				event.operationPosition = insertIndex;
				
				// Insert new edges.
				event.segmentIndexList = preList;
				event.segmentIndexList.insert(event.operationPosition, i1);
				event.segmentIndexList.insert(event.operationPosition, i0);
				
				// Update segment order.
				segmentOrder[i0] = insertIndex;
				segmentOrder[i1] = insertIndex + 1;				
				for (int k = insertIndex; k < (int)preList.size(); ++k)
				{
					segmentOrder[preList[k]] += 2;
				}
			}
		}
		else if (vertexIndex == startIndex[i1])
		{
			// i0 ends, i1 starts.
			event.eventType = Replace;

			// Get the operation position.
			event.operationPosition = segmentOrder[i0];

			// Update current segment list.
			const Array<int>& preList = mEventList[i-1].segmentIndexList;
			event.segmentIndexList = preList;
			event.segmentIndexList[event.operationPosition] = i1;

			// Update segment order.
			segmentOrder[i0] = -1;
			segmentOrder[i1] = event.operationPosition;
		}
		else if (vertexIndex == startIndex[i0])
		{
			// i1 ends, i0 starts.
			event.eventType = Replace;

			// Get the operation position.
			event.operationPosition = segmentOrder[i1];

			// Update current segment list.
			const Array<int>& preList = mEventList[i-1].segmentIndexList;
			event.segmentIndexList = preList;
			event.segmentIndexList[event.operationPosition] = i0;

			// Update segment order.
			segmentOrder[i1] = -1;
			segmentOrder[i0] = event.operationPosition;
		}
		else
		{
			// Two segments end.
			event.eventType = Remove;

			// Get the operation position.
			event.operationPosition = segmentOrder[i0] < segmentOrder[i1] ? segmentOrder[i0] : segmentOrder[i1];

			// Remove the two edges in the current segment list.
			const Array<int>& preList = mEventList[i-1].segmentIndexList;
			event.segmentIndexList = preList;
			event.segmentIndexList.erase(event.segmentIndexList.begin() + event.operationPosition);
			event.segmentIndexList.erase(event.segmentIndexList.begin() + event.operationPosition);

			// Update segment order.
			segmentOrder[i0] = -1;
			segmentOrder[i1] = -1;
			for (int k = event.operationPosition + 2; k < (int)preList.size(); ++k)
			{
				segmentOrder[preList[k]] -= 2;
			}
		}
	}

	// Initialize test flags.
	mSegmentTestFlags.Initialize(mCount);

	// Release.
	delete [] sortedVertexIndex;
	delete [] vertexOrder;
	delete [] startIndex;
	delete [] segmentOrder;

	return true;
}

void BentleyOttmann::QuickSort(int start, int end, int arr[])
{
	if (start >= end)
		return;
	
	int low = start;
	int high = end;
	int pivot = low;
	int pivotIndex = arr[pivot];

	while (low < high)
	{
		// Remember, this statements should always before the next one.
		// Since pivot is tht same to the low rather than the high at the beginning.
		// There's something waste, when start == end - 1.
		for (; low < high && mVertices[arr[high]].x > mVertices[pivotIndex].x; --high);
		arr[low] = arr[high];
		
		for (; low < high && mVertices[pivotIndex].x > mVertices[arr[low]].x; ++low);
		arr[high] = arr[low];	
	}

	pivot = low;
	arr[pivot] = pivotIndex;

	// Recur.
	QuickSort(start, pivot - 1, arr);
	QuickSort(pivot + 1, end, arr);
}

void BentleyOttmann::ReleaseBentleyOttmann(BentleyOttmann* value)
{
	delete value;
}

bool BentleyOttmann::IsPointInPolygon(const Vector2& point)
{
	// Find the event just after the given point.
	int currentEvent = 0;
	for (; currentEvent < mCount && point.x > mEventList[currentEvent].vertex.x; ++currentEvent);
	if (currentEvent == 0 || currentEvent == mCount)
        return false;

	const Array<int>& edgeList = mEventList[currentEvent - 1].segmentIndexList;
	bool isInside = false;
	for (int i = 0; i < (int)edgeList.size(); ++i)
	{
		const Segment& segment = mSegments[edgeList[i]];
		if (Segment::IsPointBelowSegment(point, segment))
			break;
		isInside = !isInside;
	}

	return isInside;
}

bool BentleyOttmann::IsSegmentIntersectPolygon(const Vector2& p0, const Vector2& p1)
{
	// Clear test flags.
	mSegmentTestFlags.Clear();

    // Create segment by p0, p1.
	Segment s;
    if (p0.x <= p1.x)
    {
		s.leftVertex = p0;
		s.rightVertex = p1;
    }
    else
	{
		s.leftVertex = p1;
		s.rightVertex = p0;
    }
	s.delta = s.rightVertex - s.leftVertex;

    // Find the start event.
    int currentEvent = 0;
	for (; currentEvent < mCount && s.leftVertex.x > mEventList[currentEvent].vertex.x; ++currentEvent);
    if (currentEvent == mCount)
        return false;

	// Test the given segment to the edge list of the start event.
    int segmentPosition = 0;    
    if (currentEvent > 0)
    {
		// Find the position of the given segment in the edge list of the start event.
        const Array<int>& currentEdgeList = mEventList[currentEvent - 1].segmentIndexList;
        for (segmentPosition = 0; segmentPosition < (int)currentEdgeList.size(); ++segmentPosition)
        {
			const Segment& segment = mSegments[currentEdgeList[segmentPosition]];
			if (Segment::IsPointBelowSegment(s.leftVertex, segment))
                break;
        }

        // Do intersection test.
        if (segmentPosition > 0)
        {
            // Test against currentEdgeList[segmentPosition - 1], which is below.
			int segmentIndex = currentEdgeList[segmentPosition - 1];
            const Segment& segment = mSegments[segmentIndex];
			if (Segment::IsSegmentIntersect(segment, s, true))
                return true;

			// Set the tested flag.
			mSegmentTestFlags.SetBit(segmentIndex);
        }

        if (segmentPosition < (int)currentEdgeList.size())
        {
            // Test against currentEdgeList[segmentPosition], which is above.
			int segmentIndex = currentEdgeList[segmentPosition];
            const Segment& segment = mSegments[segmentIndex];
			if (Segment::IsSegmentIntersect(segment, s, false))
                return true;

			// Set the tested flag.
			mSegmentTestFlags.SetBit(segmentIndex);
        }
    }

    // Loop the events.
    for (; currentEvent < mCount; ++currentEvent)
    {
		// Test x of the next event.
		if (s.rightVertex.x < mEventList[currentEvent].vertex.x)
            break;

        const Event& event = mEventList[currentEvent];
        switch (event.eventType)
        {
        case Insert:
            {
                if (event.operationPosition < segmentPosition)
                {
                    // Insert below the below neighbour of the given segment, 
					// so no new neighbour introduced, just update the segment position.
					segmentPosition += 2;
                }
                else if (event.operationPosition == segmentPosition)
                {
					if (Segment::IsPointBelowSegment(event.vertex, s))
                    {
						// If the start point of the two new edge is below the given segment,
						// update the segment position.
                        segmentPosition += 2;

                        // Test intersection against the second segment, which is below.
						int segmentIndex = event.segmentIndexList[event.operationPosition + 1];
                        Segment& segment = mSegments[segmentIndex];
						if (Segment::IsSegmentIntersect(segment, s, true))
                            return true;

						// Set the tested flag.
						mSegmentTestFlags.SetBit(segmentIndex);
                    }
                    else
                    {
						// The two new edge are inserted above the given segment.
                        // Test intersection against the first segment.
						int segmentIndex = event.segmentIndexList[event.operationPosition];
                        Segment& segment = mSegments[segmentIndex];
                        if (Segment::IsSegmentIntersect(segment, s, false))
                            return true;

						// Set the tested flag.
						mSegmentTestFlags.SetBit(segmentIndex);
                    }
                }
            }
            break;
        case Replace:
            {
                if (event.operationPosition == segmentPosition - 1 ||
					event.operationPosition == segmentPosition) 
                {
                    // Test intersection against the new segment below/above.
					int segmentIndex = event.segmentIndexList[event.operationPosition];
                    Segment& segment = mSegments[segmentIndex];
					bool isBelow = event.operationPosition == segmentPosition - 1;
					if (Segment::IsSegmentIntersect(segment, s, isBelow))
                        return true;

					// Set the tested flag.
					mSegmentTestFlags.SetBit(segmentIndex);
                }
            }
            break;
        case Remove:
            {
                if (event.operationPosition == segmentPosition - 2)
                {
                    // Remove the two segments just below the given segment.
					// Update the segment opsition.
                    segmentPosition -= 2;

                    // Test intersection to the new neighbour below.
                    if (segmentPosition > 0)
                    {
                        Event& preEvent = mEventList[currentEvent - 1];
						int segmentIndex = preEvent.segmentIndexList[event.operationPosition - 1];
                        Segment& segment = mSegments[segmentIndex];
						if (!mSegmentTestFlags.CheckBit(segmentIndex) && 
							Segment::IsSegmentIntersect(segment, s, true))
                            return true;

						// Set the tested flag.
						mSegmentTestFlags.SetBit(segmentIndex);
                    }
                }
                else if (event.operationPosition == segmentPosition)
                {
                    // Remove the two edges just above the given segment. 
					// Test intersection to the new neighbour above.
                    if (segmentPosition < (int)event.segmentIndexList.size())
                    {
                        Event& preEvent = mEventList[currentEvent - 1];
						int segmentIndex = preEvent.segmentIndexList[event.operationPosition + 2];
                        Segment& segment = mSegments[segmentIndex];
                        if (!mSegmentTestFlags.CheckBit(segmentIndex) && 
							Segment::IsSegmentIntersect(segment, s, false))
                            return true;

						// Set the tested flag.
						mSegmentTestFlags.SetBit(segmentIndex);
                    }
                }
            }
            break;
		}
    }

    return false;
}