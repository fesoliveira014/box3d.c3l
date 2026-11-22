// The harness scripts/tsan-check.sh runs under ThreadSanitizer: a 256-box pile on four workers with
// the contact material callback set (and its contacts invalidated every 20 steps) or not, and the
// origin shifted every 30 steps or not.
#include "box3d/box3d.h"
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>

static _Atomic int g_calls;

// A plain counter the racy mode increments, to show the sanitizer sees a race in the callback.
static int g_racy;
static int g_beRacy;

static void Materials( b3ShapeId a, b3ShapeId b, b3ContactMaterial* material, void* context )
{
	const float* table = context;
	atomic_fetch_add_explicit( &g_calls, 1, memory_order_relaxed );
	material->friction = table[( a.index1 + b.index1 ) & 1];
	material->restitution = 0.1f;
	if ( g_beRacy )
	{
		g_racy += 1;
	}
}

int main( int argc, char** argv )
{
	int useCallback = argc > 1 && atoi( argv[1] ) != 0;
	int useShift = argc > 2 && atoi( argv[2] ) != 0;
	g_beRacy = argc > 3 && atoi( argv[3] ) != 0;
	static float table[2] = { 0.2f, 0.9f };

	b3WorldDef def = b3DefaultWorldDef();
	def.workerCount = 4;
	b3WorldId world = b3CreateWorld( &def );
	if ( useCallback )
	{
		b3World_SetContactMaterialCallback( world, Materials, table );
	}

	b3BodyDef bodyDef = b3DefaultBodyDef();
	bodyDef.position = ( b3Pos ){ 5000.0f, -0.5f, 0.0f };
	b3BodyId ground = b3CreateBody( world, &bodyDef );
	b3BoxHull groundBox = b3MakeBoxHull( 20.0f, 0.5f, 20.0f );
	b3ShapeDef shapeDef = b3DefaultShapeDef();
	b3CreateHullShape( ground, &shapeDef, &groundBox.base );

	b3BoxHull box = b3MakeBoxHull( 0.3f, 0.3f, 0.3f );
	for ( int i = 0; i < 256; ++i )
	{
		bodyDef = b3DefaultBodyDef();
		bodyDef.type = b3_dynamicBody;
		bodyDef.position = ( b3Pos ){ 5000.0f + (float)( i % 16 ) - 7.5f, 0.3f + (float)( i / 64 ) * 0.7f, (float)( ( i / 16 ) % 4 ) - 1.5f };
		bodyDef.enableContactRecycling = false;
		b3BodyId body = b3CreateBody( world, &bodyDef );
		b3CreateHullShape( body, &shapeDef, &box.base );
	}

	for ( int step = 0; step < 240; ++step )
	{
		if ( useCallback && step % 20 == 10 )
		{
			b3World_InvalidateContactMaterials( world );
		}
		if ( useShift && step % 30 == 29 )
		{
			b3World_ShiftOrigin( world, ( b3Pos ){ step % 60 == 59 ? -5000.0f : 5000.0f, 0.0f, 0.0f } );
		}
		b3World_Step( world, 1.0f / 60.0f, 4 );
	}

	printf( "callback %d shift %d calls %d awake %d\n", useCallback, useShift, atomic_load( &g_calls ), b3World_GetAwakeBodyCount( world ) );
	b3DestroyWorld( world );
	return 0;
}
